#!/usr/bin/env python

import rospy
import actionlib
import PyKDL
import math
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

waypoints = []

def put_to_waypoints(name, pose, orientation):
	new_quat = quaternion_from_euler(*orientation)
	waypoints.append([name, pose, new_quat])

def trigger_callback(msg):
	global wait
	if msg.data == True:
		wait = False


class Waypoint(State):
	def __init__(self, position, orientation):
		State.__init__(self, outcomes=['success'])

		# Get an action client
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

		# Define the goal
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = position[0]
		self.goal.target_pose.pose.position.y = position[1]
		self.goal.target_pose.pose.position.z = 0.0
		self.goal.target_pose.pose.orientation.x = orientation[0]
		self.goal.target_pose.pose.orientation.y = orientation[1]
		self.goal.target_pose.pose.orientation.z = orientation[2]
		self.goal.target_pose.pose.orientation.w = orientation[3]

	def execute(self, userdata):
		self.client.send_goal(self.goal)
		self.client.wait_for_result()
		return 'success'


if __name__ == '__main__':
	global wait
	wait = True
	rospy.init_node('patrol')
	rospy.Subscriber('bag_recorder/ready', Bool, trigger_callback)
	bag_finisher = rospy.Publisher('bag_recorder/finished', Bool, queue_size=1)

	print('waititng')
	# while wait:
	# 	rospy.sleep(1)

	# put_to_waypoints('one', (0.0, 0.0), (0.0, 0.0, math.pi/2))

	# 	long line
	# put_to_waypoints('one', (0.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('two', (3.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('three', (0.0, 0.0), (0.0, 0.0, 0.0))

	# straight line
	# put_to_waypoints('one', (0.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('two', (0.0, 1.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('three', (0.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('four', (1.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('five', (0.0, 0.0), (0.0, 0.0, 0.0))

	# circle
	# put_to_waypoints('one', (0.0, 0.0), (0.0, 0.0, 2.0))
	# put_to_waypoints('one_half', (0.0, 0.0), (0.0, 0.0, 3.0))
	# put_to_waypoints('two', (0.0, 0.0), (0.0, 0.0, 4.0))
	# put_to_waypoints('three', (0.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('four', (0.0, 0.0), (0.0, 0.0, 4.0))
	# put_to_waypoints('four_half', (0.0, 0.0), (0.0, 0.0, 3.0))
	# put_to_waypoints('five', (0.0, 0.0), (0.0, 0.0, 2.0))
	# put_to_waypoints('six', (0.0, 0.0), (0.0, 0.0, 0.0))


	# square with rotation
	put_to_waypoints('one', (0.0, 0.0), (0.0, 0.0, 0.0))
	put_to_waypoints('two', (0.0, 0.0), (0.0, 0.0, -math.pi/2))
	put_to_waypoints('three', (0.0, -1.0), (0.0, 0.0, -math.pi/2))
	put_to_waypoints('four', (0.0, -1.0), (0.0, 0.0, 0.0))
	put_to_waypoints('five', (2.0, -1.0), (0.0, 0.0, 0.0))
	put_to_waypoints('six', (2.0, -1.0), (0.0, 0.0, math.pi/2))
	put_to_waypoints('seven', (2.0, 1.0), (0.0, 0.0, math.pi / 2))
	put_to_waypoints('eight', (2.0, 1.0), (0.0, 0.0, math.pi))
	put_to_waypoints('nine', (0.0, 1.0), (0.0, 0.0, math.pi))
	put_to_waypoints('ten', (0.0, 1.0), (0.0, 0.0, -math.pi/2))
	put_to_waypoints('eleven', (0.0, 0.0), (0.0, 0.0, -math.pi / 2))
	# put_to_waypoints('four', (1.0, 1.0), (0.0, 0.0, math.pi))
	# put_to_waypoints('five', (0.0, 1.0), (0.0, 0.0, math.pi))
	# put_to_waypoints('six', (0.0, 1.0), (0.0, 0.0, math.pi * 3 / 2))
	# put_to_waypoints('seven', (0.0, 0.0), (0.0, 0.0, -math.pi / 2))
	# put_to_waypoints('eight', (0.0, 0.0), (0.0, 0.0, 0.0))

	# square with constant orientation
	# put_to_waypoints('start', (0.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('one', (1.0, 0.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('three', (1.0, 1.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('five', (0.0, 1.0), (0.0, 0.0, 0.0))
	# put_to_waypoints('eight', (0.0, 0.0), (0.0, 0.0, 0.0))


	patrol = StateMachine(outcomes=['finished', 'success'])
	with patrol:
		for i,w in enumerate(waypoints):
			if i == len(waypoints) - 1:
				StateMachine.add(w[0],
								 Waypoint(w[1], w[2]),
								 transitions={'success': 'finished'})
			else:
				StateMachine.add(w[0],
							 Waypoint(w[1], w[2]),
							 transitions={'success':waypoints[i + 1][0]})

	print("executing")
	patrol.execute()
	bag_finisher.publish(True)
