#!/usr/bin/env python

# translate layer for lalkarz node that rotates movement of commands

import rospy
from geometry_msgs.msg import Twist

class RotateManager:
    def __init__(self, pub_topic):
        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=1)

    def rotate_callback(self, twist):
        new_twist = twist
        new_twist.linear.x, new_twist.linear.y = new_twist.linear.y, -new_twist.linear.x
        self.pub.publish(new_twist)


if __name__ == '__main__':

    rospy.init_node("lalkarz_translate")
    manager = RotateManager('/cmd_vel')

    rospy.Subscriber('/lalkarz/cmd_vel', Twist, manager.rotate_callback)
    rospy.spin()
