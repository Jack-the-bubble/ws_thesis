#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from tf.transformations import euler_from_quaternion

class OdomError:
    def __init__(self):
        self.calculated_pose = Odometry()
        self.real_pose = GetModelStateResponse()
        self.abs_x_err = 0
        self.abs_y_err = 0
        self.abs_theta_err = 0

    def calc_callback(self, odom):
        self.calculated_pose = odom

    def real_callback(self, odom):
        self.real_pose = odom

    def abs_error_x(self):
        if self.calculated_pose and self.real_pose:
            self.abs_x_err = abs(self.calculated_pose.pose.pose.position.x - self.real_pose.pose.position.x)
            return self.abs_x_err
        else:
            return ""

    def abs_error_y(self):
        if self.calculated_pose and self.real_pose:
            self.abs_y_err = abs(self.calculated_pose.pose.pose.position.y - self.real_pose.pose.position.y)
            return self.abs_y_err
        else:
            return ""

    def abs_error_angle(self):
        if self.calculated_pose and self.real_pose:
            calc_quat = [self.calculated_pose.pose.pose.orientation.x, self.calculated_pose.pose.pose.orientation.y,
                         self.calculated_pose.pose.pose.orientation.z, self.calculated_pose.pose.pose.orientation.w]
            _, _, calc_rpy = euler_from_quaternion(calc_quat)
            real_quat = [self.real_pose.pose.orientation.x, self.real_pose.pose.orientation.y,
                         self.real_pose.pose.orientation.z, self.real_pose.pose.orientation.w]
            _, _, real_rpy = euler_from_quaternion(real_quat)
            self.abs_theta_err = abs(calc_rpy - real_rpy)
            return self.abs_theta_err
        else:
            return ""

    def calc_errors(self):
        print("calculated error is: \n x: {}\n y: {}\n angle: {}".format(self.abs_error_x(), self.abs_error_y(),
                                                                         self.abs_error_angle()))
        # print("calculated pose is {}, \n real pose is {}".format(self.calculated_pose, self.real_pose))

if __name__ == '__main__':
    error_handler = OdomError()
    model = GetModelStateRequest()
    model.model_name = 'velma'



    rospy.init_node("odom_errors", anonymous=True)
    rospy.Subscriber("odom", Odometry, error_handler.calc_callback)
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        result = get_model_srv(model)
        error_handler.real_callback(result)
        error_handler.calc_errors()
        r.sleep()
