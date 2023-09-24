#!/usr/bin/env python3

from copy import deepcopy
from math import sqrt
from threading import Lock

import rospy
from geometry_msgs.msg import Point, Pose
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, Empty, EmptyRequest, EmptyResponse

from marker_interfacing.msg import ENUMarker
from aruco_finder.msg import FoundMarkerList

### helpers ##################################################################

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

		self.gate_square_side_length = rospy.get_param("~gate_square_side_length")

		self.pose_lock = Lock()
		self.current_pose = None

		self.marker_lock = Lock()
		self.aruco_id = -1

		self.gate_offsets_lock = Lock()
		self.gate_offsets = []

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

	def plan_gate_offsets(self, current_position, marker_position):
		# gate marker is *
		# robot will start at nearest corner and follow numbers in counterclockwise manner
		# 2-------1
		# |       |
		# |   *   |
		# |       |
		# 3-------0

		#                     x                                 y                              z
		bottom_right = Point( self.gate_square_side_length / 2, -self.gate_square_side_length, 0)
		top_right    = Point( self.gate_square_side_length / 2,  self.gate_square_side_length, 0)
		top_left     = Point(-self.gate_square_side_length / 2,  self.gate_square_side_length, 0)
		bottom_left  = Point(-self.gate_square_side_length / 2, -self.gate_square_side_length, 0)

		gate_offsets_default_order = [bottom_right, top_right, top_left, bottom_left]

		# find index of nearest corner
		left = current_position.x < marker_position.x
		below = current_position.y < marker_position.y
		right, above = not left, not below
		if right and below:
			nearest_corner_index = 0
		elif right and above:
			nearest_corner_index = 1
		elif left and above:
			nearest_corner_index = 2
		elif left and below:
			nearest_corner_index = 3

		nearest_marker = [gate_offsets_default_order[nearest_corner_index]]
		markers_after_nearest_marker_inclusive = gate_offsets_default_order[nearest_corner_index:]
		markers_before_nearest_marker_exclusive = gate_offsets_default_order[:nearest_corner_index]
		gate_offsets_reordered = markers_after_nearest_marker_inclusive + markers_before_nearest_marker_exclusive + nearest_marker

		with self.gate_offsets_lock:
			self.gate_offsets.clear()
			self.gate_offsets += gate_offsets_reordered

	### callbacks ############################################################

	def pose_sub_callback(self, pose_msg: Pose):
		with self.pose_lock:
			self.current_pose = pose_msg

	def current_gate_sub_callback(self, enu_marker: ENUMarker):
		with self.marker_lock:
			self.aruco_id = enu_marker.aruco_id
			self.aruco_id_2 = enu_marker.aruco_id_2
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
		with self.gate_offsets_lock:
			current_gate_offset = deepcopy(self.gate_offsets[0]) if len(self.gate_offsets) > 0 else None
		with self.marker_lock:
			aruco_id = deepcopy(self.aruco_id)
			aruco_id_2 = deepcopy(self.aruco_id_2)
		with self.found_marker_dict_lock:
			found_marker_enu_1 = deepcopy(self.found_marker_dict.get(aruco_id, None))
			found_marker_enu_2 = deepcopy(self.found_marker_dict.get(aruco_id_2, None))

		found_marker_enu = found_marker_enu_1 or found_marker_enu_2
		
		if current_gate_offset and found_marker_enu:

			# add offset to found_marker_enu
			current_waypoint = Point()
			current_waypoint.x = found_marker_enu.x + current_gate_offset.x
			current_waypoint.y = found_marker_enu.y + current_gate_offset.y
			current_waypoint.z = found_marker_enu.z + current_gate_offset.z

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

		# check if neither aruco_id
		with self.marker_lock:
			aruco_id = deepcopy(self.aruco_id)
			aruco_id_2 = deepcopy(self.aruco_id_2)
		if aruco_id < 0 and aruco_id_2 < 0:
			msg = "Circle Gate Planner does not have either marker (aruco_id)! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response

		# check if marker not found
		with self.found_marker_dict_lock:
			found_marker_enu_1 = deepcopy(self.found_marker_dict.get(aruco_id, None))
			found_marker_enu_2 = deepcopy(self.found_marker_dict.get(aruco_id_2, None))
		if not found_marker_enu_1 and not found_marker_enu_2:
			msg = f"Circle Gate Planner does not have position of aruco ids {aruco_id}, {aruco_id_2}! Cannot set next waypoint!"
			rospy.logwarn(msg)
			response.success = False
			response.message = msg
			return response

		with self.gate_offsets_lock:
			has_current_offset = len(self.gate_offsets) > 0

			# remove current gate offset from list, if any
			if has_current_offset:
				self.gate_offsets.pop(0)

		# plan new list of offsets, if none
		if not has_current_offset:
			with self.pose_lock:
				current_position = self.current_pose.position
			found_marker_enu = found_marker_enu_1 or found_marker_enu_2
			self.plan_gate_offsets(current_position, found_marker_enu)

		# check if complete
		with self.gate_offsets_lock:
			if len(self.gate_offsets) == 0:
				with self.complete_lock:
					self.complete = True

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
