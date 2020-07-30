#!/usr/bin/env python

import os
import math

import rospy
import rosbag
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
	bag_prefix = '/home/silver/INZYNIER/velma/ws_thesis/src/' \
	             'velma_experiments/data'

	name = 'teb_laser_vel03_square_one_orientation.bag'

	bag_name = os.path.join(bag_prefix, name)

	bag = rosbag.Bag(bag_name)
	odom_count = bag.get_message_count('odom')
	real_pos_count = bag.get_message_count('ground_truth/state')

	odom_list = []
	real_pos_list = []
	error_dict = {'x': [], 'y':[], 'orientation':[]}

	for odom_msgs in bag.read_messages('odom'):
		odom_list.append(odom_msgs)

	for real_msgs in bag.read_messages('ground_truth/state'):
		real_pos_list.append(real_msgs)

	for odom, real in zip(odom_list, real_pos_list):
		test = Odometry()
		# test.pose.pose.orientation.
		odom_rpy = euler_from_quaternion(*odom.pose.pose.orientaiton)
		real_rpy = euler_from_quaternion(*real.pose.pose.orientaiton)
		error_dict['x'].append(abs(odom.pose.pose.position.x - real.pose.pose.position.x))
		error_dict['y'].append(abs(odom.pose.pose.position.y - real.pose.pose.position.y))
		error_dict['x'].append(abs(odom_rpy[2] - real_rpy[2]))