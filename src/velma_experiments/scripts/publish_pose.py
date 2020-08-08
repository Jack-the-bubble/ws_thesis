#!/usr/bin/env python

import math
import rospy
import sys

from geometry_msgs.msg import Pose2D, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import action_client, SimpleActionClient
from tf.transformations import quaternion_from_euler

def move_base_client():

	client = SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()

	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()

	# line - 3, -1, -math.pi/2
	theta = eval(sys.argv[3])
	quat = quaternion_from_euler(0, 0, theta)
	goal.target_pose.pose.position.x = float(sys.argv[1])
	goal.target_pose.pose.position.y = float(sys.argv[2])
	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]

	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('nav_pose_publisher')

		result = move_base_client()

		if result:
			rospy.loginfo("Goal execution done!")

	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
