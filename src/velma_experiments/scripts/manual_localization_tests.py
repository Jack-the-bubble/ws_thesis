#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from tf.transformations import euler_from_quaternion
import math

class ManualNav:
	def __init__(self):
		self.goals_sequence = []
		self.current_position = Pose2D()
		self.pose_reached = False
		self.nav_sleep = 0.1
		self.lin_tolerance = 0.2
		self.angular_tolerance = 0.2
		self.max_lin_vel = 0.3
		self.max_ang_vel = 0.5
		self.vel_acc = 0.05
		self.current_twist = Twist()
		# self.calc_ = Pose2D()
		# self.calc_ = PoseWithCovarianceStamped()

		rospy.Subscriber('odom', Odometry, self.odom_callback)
		# rospy.Subscriber('pose2D', Pose2D, self.laser_callback)
		# rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.odom_callback)

		self.vel_pub =rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def complete_sequence(self):
		for position in self.goals_sequence:
			print("moving to next point")
			self.pose_reached = False
			while not self.pose_reached:
				self.current_twist = self.calculate_velocities(position)
				self.vel_pub.publish(self.current_twist)
				rospy.sleep(self.nav_sleep)

	def calculate_velocities(self, pos):
		next_vel = Twist()

		tmp_theta = pos.theta
		if pos.theta < 0:
			tmp_theta = 2 * math.pi + pos.theta

		tmp_current_theta = self.current_position.theta
		if self.current_position.theta < 0:
			tmp_current_theta = 2 * math.pi + self.current_position.theta

		theta_dist = (tmp_theta - tmp_current_theta) % (2 * math.pi)

		if theta_dist > math.pi:
			theta_dist = -2 * math.pi + theta_dist
		elif theta_dist < - math.pi:
			theta_dist = 2 * math.pi + theta_dist


		if theta_dist > self.angular_tolerance:
			next_vel.angular.z = min(self.current_twist.angular.z + self.vel_acc, self.max_ang_vel)
		elif theta_dist < -self.angular_tolerance:
			next_vel.angular.z = max(self.current_twist.angular.z - self.vel_acc, -self.max_ang_vel)
		else:
			next_vel.angular.z = 0


		x_dist = pos.x - self.current_position.x
		if x_dist > self.lin_tolerance:
			next_vel.linear.x = min(self.current_twist.linear.x + self.vel_acc, self.max_lin_vel)
		elif x_dist < -self.lin_tolerance:
			next_vel.linear.x = max(self.current_twist.linear.x - self.vel_acc, -self.max_lin_vel)
		else:
			next_vel.linear.x = 0

		y_dist = pos.y - self.current_position.y
		if y_dist > self.lin_tolerance:
			next_vel.linear.y = min(self.current_twist.linear.y + self.vel_acc, self.max_lin_vel)
		elif y_dist < -self.lin_tolerance:
			next_vel.linear.y = max(self.current_twist.linear.y - self.vel_acc, -self.max_lin_vel)
		else:
			next_vel.linear.y = 0

		if next_vel.linear.x == 0 and next_vel.linear.y == 0 and next_vel.angular.z == 0:
			self.pose_reached = True

		return next_vel


	def odom_callback(self, msg):
		orient_ = msg.pose.pose.orientation
		_, _, yaw = euler_from_quaternion([orient_.x, orient_.y, orient_.z, orient_.w])
		self.current_position.x = msg.pose.pose.position.x
		self.current_position.y = msg.pose.pose.position.y
		self.current_position.theta = yaw
		print("Theta: {}".format(yaw))

	def laser_callback(self, msg):
		self.current_position = msg

	# def amcl_callback(self, msg):
	# 	self.calc_ = msg


if __name__ == '__main__':
	rospy.init_node('manual_localization_tests')

	simple_planner = ManualNav()

	# long line
	simple_planner.goals_sequence.append(Pose2D(0, 0, 0))
	simple_planner.goals_sequence.append(Pose2D(3, 0, 0))
	simple_planner.goals_sequence.append(Pose2D(0, 0, 0))

	# circle
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 0))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 2))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 4))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 2*math.pi))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 4))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 2))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 0))

	# square const orientation
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 0))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, 0))
	# simple_planner.goals_sequence.append(Pose2D(1, 1, 0))
	# simple_planner.goals_sequence.append(Pose2D(0, 1, 0))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 0))

	# square with rotation
	# simple_planner.goals_sequence.append(Pose2D(0, 0, 0))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, 0))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, math.pi / 2))
	# simple_planner.goals_sequence.append(Pose2D(1, 1, math.pi / 2))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, math.pi))
	# simple_planner.goals_sequence.append(Pose2D(0, 1, math.pi))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, math.pi * 3 / 2))
	# simple_planner.goals_sequence.append(Pose2D(0, 0, math.pi * 3 / 2))
	# simple_planner.goals_sequence.append(Pose2D(1, 0, 2 * math.pi))


	simple_planner.complete_sequence()