#!/usr/bin/env python3

from copy import deepcopy
from enum import Enum, auto
from math import sin, cos, atan2, sqrt
from threading import Lock

import rospy
from geometry_msgs.msg import Point, Pose
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, Empty, EmptyRequest, EmptyResponse

from marker_interfacing.msg import ENUMarker
from aruco_finder.msg import FoundMarkerList

### helpers ##################################################################

class WaypointDescriptor(Enum):
	null = auto()
	near_point = auto()
	center_point = auto()
	far_point = auto()

def dist(pt1: Point, pt2: Point):
	x_err = pt1.x - pt2.x
	y_err = pt1.y - pt2.y
	z_err = pt1.z - pt2.z
	return sqrt(x_err ** 2 + y_err ** 2 + z_err ** 2)

### main #####################################################################

def main():
	GatePlanner().loop()

class GatePlanner:
	def __init__(self) -> None:
		rospy.init_node('gate_planner')

		### local variables ##################################################

		self.gate_path_length = rospy.get_param("~gate_path_length")

		self.pose_lock = Lock()
		self.current_pose = None
		self.position_at_beginning_of_gate_traversal = None

		self.marker_lock = Lock()
		self.aruco_id = -1

		self.waypoint_descriptors_lock = Lock()
		self.waypoint_descriptors = []

		self.found_marker_dict_lock = Lock()
		self.found_marker_dict = {}

		self.complete_lock = Lock()
		self.complete = False

		### connect to ROS ###################################################

		local_position_topic = rospy.get_param("~local_position_topic")
		current_gate_topic = rospy.get_param("~current_gate_topic")
		found_marker_list_topic = rospy.get_param("~found_marker_list_topic")
		current_waypoint_topic = rospy.get_param("~current_waypoint_topic")
		next_waypoint_service = rospy.get_param("~next_waypoint_service")
		update_waypoint_service = rospy.get_param("~update_waypoint_service")

		self.pose_sub = rospy.Subscriber(local_position_topic, Pose, self.pose_sub_callback)
		self.current_gate_sub = rospy.Subscriber(current_gate_topic, ENUMarker, self.current_gate_sub_callback)
		self.found_marker_list_sub = rospy.Subscriber(found_marker_list_topic, FoundMarkerList, self.found_marker_list_sub_callback)
		self.current_waypoint_pub = rospy.Publisher(current_waypoint_topic, Point, queue_size=1)
		self.next_waypoint_srv = rospy.Service(next_waypoint_service, Trigger, self.next_waypoint_callback)
		self.update_waypoint_srv = rospy.Service(update_waypoint_service, Empty, self.update_waypoint_callback)

		### end init #########################################################

	### local functions ######################################################

	def plan_waypoint_descriptors(self):
		# gate markers are *
		# robot will start at 0 and go to 1
		#          *   
		#       0-----1
		#          *   

		with self.waypoint_descriptors_lock:
			self.waypoint_descriptors.clear()
			self.waypoint_descriptors.append(WaypointDescriptor.null)  # require caller to ask for first waypoint
			self.waypoint_descriptors.append(WaypointDescriptor.near_point)
			self.waypoint_descriptors.append(WaypointDescriptor.center_point)
			self.waypoint_descriptors.append(WaypointDescriptor.far_point)

	def calculate_path_points(self, found_marker_enu, found_marker_enu_2):
		# find angle of target path
		dy = (found_marker_enu.y - found_marker_enu_2.y)
		dx = (found_marker_enu.x - found_marker_enu_2.x)
		path_angle = atan2(-dx, dy)  # perpendicular to angle between markers

		# find x and y deltas for points {path_to_point_distance} distance from center on the path
		center_to_point_distance = self.gate_path_length / 2
		center_to_point_dy = sin(path_angle) * center_to_point_distance
		center_to_point_dx = cos(path_angle) * center_to_point_distance

		# find center
		center = Point()
		center.x = (found_marker_enu.x + found_marker_enu_2.x) / 2
		center.y = (found_marker_enu.y + found_marker_enu_2.y) / 2
		center.z = (found_marker_enu.z + found_marker_enu_2.z) / 2

		# calculate points on path
		pt1 = Point()
		pt1.x = center.x + center_to_point_dx
		pt1.y = center.y + center_to_point_dy
		pt1.z = center.z

		pt2 = Point()
		pt2.x = center.x - center_to_point_dx
		pt2.y = center.y - center_to_point_dy
		pt2.z = center.z

		return pt1, center, pt2

	### callbacks ############################################################

	def pose_sub_callback(self, pose_msg: Pose):
		with self.pose_lock:
			self.current_pose = pose_msg

	def current_gate_sub_callback(self, enu_marker: ENUMarker):
		with self.marker_lock:
			self.aruco_id = enu_marker.aruco_id
			self.aruco_id_2 = enu_marker.aruco_id_2
		self.plan_waypoint_descriptors()
		with self.complete_lock:
			self.complete = False

	def found_marker_list_sub_callback(self, found_marker_list: FoundMarkerList):
		with self.found_marker_dict_lock:
			self.found_marker_dict = {}
			for found_marker in found_marker_list.markers:
				self.found_marker_dict[found_marker.aruco_id] = found_marker.marker_enu

	def update_waypoint(self):
		# check if complete
		with self.complete_lock:
			complete = self.complete
		if complete:
			return

		# update waypoint
		with self.waypoint_descriptors_lock:
			current_waypoint_descriptor = deepcopy(self.waypoint_descriptors[0]) if len(self.waypoint_descriptors) > 0 else None
		with self.marker_lock:
			aruco_id = deepcopy(self.aruco_id)
			aruco_id_2 = deepcopy(self.aruco_id_2)
		with self.found_marker_dict_lock:
			found_marker_enu = deepcopy(self.found_marker_dict.get(aruco_id, None))
			found_marker_enu_2 = deepcopy(self.found_marker_dict.get(aruco_id_2, None))
		
		point_descriptor = current_waypoint_descriptor and current_waypoint_descriptor != WaypointDescriptor.null
		if point_descriptor and found_marker_enu and found_marker_enu_2:

			# calculate points for path
			pt1, center, pt2 = self.calculate_path_points(found_marker_enu, found_marker_enu_2)

			# determine which point is near and which is far
			dist_to_pt1 = dist(self.position_at_beginning_of_gate_traversal, pt1)
			dist_to_pt2 = dist(self.position_at_beginning_of_gate_traversal, pt2)
			near_point, far_point = (pt1, pt2) if dist_to_pt1 < dist_to_pt2 else (pt2, pt1)

			# choose current waypoint based on waypoint descriptor
			if current_waypoint_descriptor == WaypointDescriptor.near_point:
				current_waypoint = near_point
			if current_waypoint_descriptor == WaypointDescriptor.center_point:
				current_waypoint = center
			if current_waypoint_descriptor == WaypointDescriptor.far_point:
				current_waypoint = far_point

			# publish current waypoint
			self.current_waypoint_pub.publish(current_waypoint)

	def next_waypoint_callback(self, _: TriggerRequest):
		response = TriggerResponse()
		response.success = True
		response.message = "Success"

		# check if complete
		with self.complete_lock:
			complete = self.complete
		if complete:
			response.success = True
			response.message = "Complete"
			return response

		# check if no descriptors
		with self.waypoint_descriptors_lock:
			has_waypoint_descriptor = len(self.waypoint_descriptors) > 0
		if not has_waypoint_descriptor:
			msg = "Through Gate Planner has no waypoint descriptors! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response

		# check if not both aruco_ids
		with self.marker_lock:
			aruco_id_1 = deepcopy(self.aruco_id)
			aruco_id_2 = deepcopy(self.aruco_id_2)
		if aruco_id_1 < 0 or aruco_id_2 < 0:
			msg = "Through Gate Planner does not have both markers (aruco_ids)! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response

		# check if marker not found
		with self.found_marker_dict_lock:
			found_marker_enu_1 = deepcopy(self.found_marker_dict.get(aruco_id_1, None))
			found_marker_enu_2 = deepcopy(self.found_marker_dict.get(aruco_id_2, None))
		if not found_marker_enu_1:
			msg = f"Through Gate Planner does not have position of aruco id {aruco_id_1}! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response
		if not found_marker_enu_2:
			msg = f"Through Gate Planner does not have position of aruco id {aruco_id_2}! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response

		# remove current waypoint from list
		with self.waypoint_descriptors_lock:
			waypoint_descriptor = self.waypoint_descriptors.pop(0)

		# check if complete
		with self.waypoint_descriptors_lock:
			if len(self.waypoint_descriptors) == 0:
				with self.complete_lock:
					self.complete = True

		# save position at beginning of gate traversal
		if waypoint_descriptor == WaypointDescriptor.null:
			with self.pose_lock:
				self.position_at_beginning_of_gate_traversal = self.current_pose.position

		# update waypoint
		self.update_waypoint()
		return response

	def update_waypoint_callback(self, _: EmptyRequest):
		self.update_waypoint()
		return EmptyResponse()

	### loop #################################################################

	def loop(self):
		rospy.spin()

if __name__ == '__main__':
	main()
