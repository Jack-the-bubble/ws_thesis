#!/usr/bin/env python

'''
this script creates a ROS node that subscribes to topics
and dumps data to a csv file
'''


import rospy
import pandas as pd
import os
import sys

from tf.transformations import euler_from_quaternion
from tf import TransformListener
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


class Writer:
    def __init__(self, file_name):
        final_topic_list = ['time',
                            'calculated_x', 'calculated_y', 'calculated_theta',
                            'ideal_x', 'ideal_y', 'ideal_theta',
                            'vel_x',  'vel_y', 'vel_theta']
        self.data_table = dict.fromkeys(final_topic_list)
        for key in final_topic_list:
            self.data_table[key] = []

        self.tf_listener = TransformListener()

        self.file_dir = file_name + '.csv'

        self.calc_ = Odometry()
        # self.calc_ = Pose2D()
        # self.calc_ = PoseWithCovarianceStamped()

        self.ideal_ = Odometry
        self.vel_ = Twist

        # rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('pose2D', Pose2D, self.laser_callback)
        # rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        rospy.Subscriber('ground_truth/state', Odometry, self.ideal_callback)
        rospy.Subscriber('cmd_vel', Twist, self.vel_callback)

    def write_data(self):
        print("Writing")
        if type(self.calc_) == Odometry:
            trans, rot = [], []
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("WARN: did not get tf world-base_link")
                return

            self.data_table['calculated_x'].append(trans[0])
            self.data_table['calculated_y'].append(trans[1])
            _, _, yaw = euler_from_quaternion(rot)
            self.data_table['calculated_theta'].append(yaw)

        elif  type(self.calc_) == Pose2D:
            self.data_table['calculated_x'].append(self.calc_.x)
            self.data_table['calculated_y'].append(self.calc_.y)
            self.data_table['calculated_theta'].append(self.calc_.theta)

        elif type(self.calc_) == PoseWithCovarianceStamped:
            self.data_table['calculated_x'].append(self.calc_.pose.pose.position.x)
            self.data_table['calculated_y'].append(self.calc_.pose.pose.position.y)
            orient_ = self.calc_.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orient_.x, orient_.y, orient_.z, orient_.w])
            self.data_table['calculated_theta'].append(yaw)
        # elif type(self.calc_) == Pose2D:
        #     self.data_table['calculated_x'].append(self.calc_.x)
        #     self.data_table['calculated_y'].append(self.calc_.y)
        #     self.data_table['calculated_theta'].append(self.calc_.theta)

        self.data_table['ideal_x'].append(self.ideal_.pose.pose.position.x)
        self.data_table['ideal_y'].append(self.ideal_.pose.pose.position.y)
        orient_ = self.ideal_.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient_.x, orient_.y, orient_.z, orient_.w])
        self.data_table['ideal_theta'].append(yaw)

        try:
            self.data_table['vel_x'].append(self.vel_.linear.x)
        except AttributeError:
            self.data_table['vel_x'].append(0)

        try:
            self.data_table['vel_y'].append(self.vel_.linear.y)
        except AttributeError:
            self.data_table['vel_y'].append(0)

        try:
            self.data_table['vel_theta'].append(self.vel_.angular.z)
        except AttributeError:
            self.data_table['vel_theta'].append(0)

        # self.data_table['vel_y'].append(self.vel_.linear.y)
        # self.data_table['vel_theta'].append(self.vel_.angular.z)

        self.data_table['time'].append(rospy.Time.now())

        data_frame = pd.DataFrame(self.data_table, columns=self.data_table.keys())
        data_frame.to_csv(self.file_dir)

    def odom_callback(self, msg):
        self.calc_ = msg
        # self.data_table['calculated_x'].append(msg.pose.pose.position.x)
        # self.data_table['calculated_y'].append(msg.pose.pose.position.y)
        # orient_ = msg.pose.pose.orientation
        # _, _, yaw = euler_from_quaternion(orient_)
        # self.data_table['calculated_theta'].append(yaw)

    def laser_callback(self, msg):
        self.calc_ = msg

    def amcl_callback(self, msg):
        self.calc_ = msg

    def ideal_callback(self, msg):
        self.ideal_ = msg

    def vel_callback(self, msg):
        self.vel_ = msg

if __name__ == '__main__':
    rospy.init_node('loc_data_saver')
    writer = Writer(sys.argv[1])
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        writer.write_data()