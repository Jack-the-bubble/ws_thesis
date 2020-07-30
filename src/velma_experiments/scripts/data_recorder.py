#!/usr/bin/env python

#

import inspect
import rospy
import rosbag
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion

class BagRecorder:
	def __init__(self, bag_name):
		self.bag_file = rosbag.Bag(bag_name, 'w')
		self.bag_dict = {}
		self.bag_open = True
		self.odom_latest = Odometry()
		self.real_latest = Odometry()
		self.trigger_publish = rospy.Publisher('bag_recorder/ready', Bool, queue_size=1)
		self.error_publish = rospy.Publisher('bag_recorder/error', Pose, queue_size=1)
		self.close_bag_sub = rospy.Subscriber('bag_recorder/finished', Bool, self.finish_callback)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.odom_sub = rospy.Subscriber('ground_truth/state', Odometry, self.real_callback)

	def finish_callback(self, msg):
		# self.bag_file.close()
		self.bag_open = False
		# self.finish_writing()

	def finish_writing(self):
		for keys in self.bag_dict.keys():
			for msg in self.bag_dict[keys]:
				self.bag_file.write(keys, msg)

		print('finished writing')
		self.bag_file.close()

	def odom_callback(self, msg):
		self.odom_latest = msg

	def real_callback(self, msg):
		self.real_latest = msg

	def write_error(self):
		if self.odom_latest and self.real_latest:
			error_pose = Pose()
			error_pose.x = self.odom_latest.pose.pose.position.x - self.real_latest.pose.pose.position.x
			error_pose.y = self.odom_latest.pose.pose.position.y - self.real_latest.pose.pose.position.y
			odom_rpy = euler_from_quaternion((self.odom_latest.pose.pose.orientation.x,
			                                 self.odom_latest.pose.pose.orientation.y,
			                                 self.odom_latest.pose.pose.orientation.z,
			                                 self.odom_latest.pose.pose.orientation.w))
			real_rpy = euler_from_quaternion((self.real_latest.pose.pose.orientation.x,
			                                 self.real_latest.pose.pose.orientation.y,
			                                 self.real_latest.pose.pose.orientation.z,
			                                 self.real_latest.pose.pose.orientation.w))
			error_pose.theta = odom_rpy[2] - real_rpy[2]
			self.error_publish.publish(error_pose)
			try:
				self.bag_file.write('position_error', error_pose, rospy.Time.now())
			except ValueError as e:
				print('trying to write to closed bag')

	# def cmd_vel(self, msg):
	# 	name = inspect.stack()[0][3]
	# 	try:
	# 		self.bag_file.write(name, msg, rospy.Time.now())
	# 	except Exception as exc:
	# 		print(exc)


def patch_me(target, topic_name, topic_type):
	method_name = topic_name.replace('/', '_')
	target.bag_dict[topic_name] = []
	def method(msg, in_target):
		try:
			# target.bag_dict[topic_name].append(msg)
			in_target.bag_file.write(topic_name, msg, rospy.Time.now())
			print ("called from", in_target)
		except ValueError as e:
			print('trying to write to closed bag')
	setattr(target, method_name, method)
	rospy.Subscriber(topic_name, topic_type, method, target)

if __name__ == '__main__':
	rospy.init_node('bag_recorder')



	bag_recorder = BagRecorder('/home/silver/INZYNIER/velma/ws_thesis/src/velma_experiments/data/teb_odom_vel03_long_line_2.bag')
	rospy.Subscriber('odom', Odometry, )

	patch_me(bag_recorder, 'cmd_vel', Twist)
	patch_me(bag_recorder, 'odom', Odometry)
	patch_me(bag_recorder, 'ground_truth/state', Odometry)
	bag_recorder.trigger_publish.publish(True)


	try:
		while not rospy.is_shutdown() and bag_recorder.bag_open:
			bag_recorder.trigger_publish.publish(True)
			rospy.sleep(0.1)
			bag_recorder.write_error()
	except KeyboardInterrupt:
		bag_recorder.bag_file.close()
