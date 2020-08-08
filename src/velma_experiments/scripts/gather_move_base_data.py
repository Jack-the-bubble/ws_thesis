#!/usr/bin/env python

'''
gather data from planers: mainly paths, and dump to .csv file
'''

import rospy
import pandas as pd
import os
import math
import sys

from tf.transformations import quaternion_from_euler
from tf import TransformListener
import tf

from nav_msgs.msg import Odometry,  Path
from geometry_msgs.msg import Pose2D, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class DataCollector:
	def __init__(self, save_path, file_name, planner_type):
		self.save_path = save_path

		self.directory = os.path.join(save_path, file_name)
		try:
			os.mkdir(self.directory)
		except OSError:
			print("directory probably already exists")
			exit(1)

		self.poses_file_name = os.path.join(self.directory, file_name+'-poses-1.csv')
		self.global_file_name = os.path.join(self.directory, file_name+'-global')
		self.local_file_name = os.path.join(self.directory, file_name+'-local')
		self.timestamp_file_name = os.path.join(self.directory, file_name+'-time')
		self.global_path = Path()
		self.local_path = Path()
		self.local_path_counter = 0
		self.global_path_counter = 0
		self.new_global_plan = True
		self.new_local_plan = True

		self.data_dict = {}
		self.global_dict = {}
		self.local_dict = {}
		self.time_dict = {}

		self.data_dict['pos-x'] = []
		self.data_dict['pos-y'] = []
		self.data_dict['pos-time'] = []

		self.global_dict['pos-x'] = []
		self.global_dict['pos-y'] = []
		self.global_dict['pos-time'] = []

		self.local_dict['pos-x'] = []
		self.local_dict['pos-y'] = []
		self.local_dict['pos-time'] = []

		self.time_dict['global_timestamps'] = []
		self.time_dict['local_timestamps'] = []


		self.pose_df = pd.DataFrame()
		self.global_df = pd.DataFrame()
		self.local_df = pd.DataFrame()
		self.df = pd.DataFrame()

		self.full_file = os.path.join(save_path, file_name)

		rospy.Subscriber('move_base/'+planner_type+'/global_plan',
		                 Path, self.global_path_callback)
		rospy.Subscriber('move_base/' + planner_type + '/local_plan',
		                 Path, self.local_path_callback)
		rospy.Subscriber('pose2D', Pose2D, self.laser_callback)

	def global_path_callback(self, path):
		self.global_path = path
		self.global_time_gen = rospy.Time.now()
		self.time_dict['global_timestamps'].append(path.header.stamp.secs)
		self.new_global_plan = True

	def local_path_callback(self, path):
		self.local_path = path
		self.local_time_gen = rospy.Time.now()
		self.time_dict['local_timestamps'].append(path.header.stamp.secs)
		self.new_local_plan = True

	def laser_callback(self, pose):
		self.data_dict['pos-x'].append(pose.x)
		self.data_dict['pos-y'].append(pose.y)
		self.data_dict['pos-time'].append(rospy.Time.now())

	def dump_data_to_csv(self):
		if self.new_global_plan:
			self.new_global_plan = False

			self.global_dict['pos-x'] = []
			self.global_dict['pos-y'] = []
			self.global_dict['pos-time'] = []

			for pose in self.global_path.poses:
				self.global_dict['pos-x'].\
					append(pose.pose.position.x)
				self.global_dict['pos-y'].\
					append(pose.pose.position.y)
				self.global_dict['pos-time'].\
					append(self.global_time_gen)

			try:
				self.global_df = pd.DataFrame(self.global_dict)
				self.global_df.to_csv(self.global_file_name + '-' + str(self.global_path_counter) + '.csv')
				self.global_path_counter = self.global_path_counter + 1
			except ValueError as msg:
				print(msg)

		if self.new_local_plan:
			self.new_local_plan = False

			self.local_dict['pos-x'] = []
			self.local_dict['pos-y'] = []
			self.local_dict['pos-time'] = []

			for pose in self.local_path.poses:
				self.local_dict['pos-x'].\
					append(pose.pose.position.x)
				self.local_dict['pos-y'].\
					append(pose.pose.position.y)
				self.local_dict['pos-time'].\
					append(self.global_time_gen)

			try:
				self.local_df = pd.DataFrame(self.local_dict)
				self.local_df.to_csv(self.local_file_name + '-' + str(self.local_path_counter) + '.csv')
				self.local_path_counter = self.local_path_counter + 1
			except ValueError as msg:
				print(msg)
		try:
			self.pose_df = pd.DataFrame(self.data_dict)
			self.pose_df.to_csv(self.poses_file_name)
		except ValueError as msg:
			print(msg)

		# save timestamps
		try:
			self.time_df = pd.DataFrame(self.time_dict['global_timestamps'])
			self.time_df['Index'] = range(1, len(self.time_dict['global_timestamps']))
			self.time_df.to_csv(self.timestamp_file_name+'-global.csv')
		except ValueError as msg:
			print(msg)

		try:
			self.time_df = pd.DataFrame(self.time_dict['local_timestamps'])
			self.time_df['Index'] = range(1, len(self.time_dict['local_timestamps']))
			self.time_df.to_csv(self.timestamp_file_name+'-local.csv')
		except ValueError as msg:
			print(msg)


		# self.df = self.data_dict[['laser-poses-x', 'laser-poses-y', 'laser-poses-time']].copy()
		# self.df['laser-poses-x'] = self.data_dict['laser-poses-x']
		# self.df['laser-poses-y'] = self.data_dict['laser-poses-y']
		# self.df['laser-poses-time'] = self.data_dict['laser-poses-time']
		#
		# self.df = pd.concat([self.df, self.global_df], ignore_index=True)
		# self.df = pd.concat([self.df, self.local_df], ignore_index=True)

		# self.df.to_csv(self.full_file)

	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.sleep(0.1)
			self.dump_data_to_csv()

if __name__ == '__main__':
	rospy.init_node('move_base_data_collector')
	dir_name = sys.argv[1]
	planner = sys.argv[2]
	data_collector = DataCollector(
		'/home/silver/INZYNIER/velma/ws_thesis/src/velma_experiments/data-nav',
		dir_name,
		planner
	)

	data_collector.main_loop()
