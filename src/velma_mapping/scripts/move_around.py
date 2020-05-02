#!/usr/bin/env python

import roslib
# roslib.load_manifest('stero_velma')
import rospy
import math
import numpy
import tf
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
import PyKDL
import tty
import sys
import select
import termios

q_map_start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

def get_key():
    global settings
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def initialize_velma():
    global velma
    velma = VelmaInterface()
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"



    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)


def move_head(q_dest, time):
    velma.moveHead(q_dest, time, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not velma.isHeadConfigurationClose(velma.getHeadCurrentConfiguration(), q_dest, 0.1):
        exitError(5)

def increment_move(factor):
    global velma
    # print ('factor is {}'.format(factor))
    conf = velma.getHeadCurrentConfiguration()
    if conf[0] + factor[0] > math.pi or conf[0] + factor[0] < -math.pi:
        tilt = conf[0]
    else:
        tilt = conf[0] + factor[0]
    if conf[1] + factor[1] > math.pi or conf[1] + factor[1] < -math.pi:
        pan = conf[1]
    else:
        pan = conf[1] + factor[1]
    return (tilt, pan)

def move_base(increment, time):
    q_map_start = {'torso_0_joint': 0.1, 'right_arm_0_joint': -0.3, 'right_arm_1_joint': -1.8,
                   'right_arm_2_joint': 1.25, 'right_arm_3_joint': 0.85, 'right_arm_4_joint': 0,
                   'right_arm_5_joint': -0.5,
                   'right_arm_6_joint': 0, 'left_arm_0_joint': 0.3, 'left_arm_1_joint': 1.8, 'left_arm_2_joint': -1.25,
                   'left_arm_3_joint': -0.85, 'left_arm_4_joint': 0, 'left_arm_5_joint': 0.5, 'left_arm_6_joint': 0}

    last_state = velma.getLastJointState()
    print('last state is {}, \n increment is {}'.format(last_state[1]['torso_0_joint'], increment))
    q_next = q_map_start
    # if last_state[1]['torso_0_joint'] + increment < math.pi and last_state[1]['torso_0_joint'] + increment > -math.pi:
    #     q_next['torso_0_joint'] = last_state[1]['torso_0_joint'] + increment

    velma.moveJoint(q_next, time, start_time=1, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not velma.isConfigurationClose(q_next, js[1], tolerance=0.1):
        exitError(10)

scale = 0.1

move_head_binding = {
    'i': (0, -1*scale),
    'k': (0, 1*scale),
    'j': (1*scale, 0),
    'l': (-1*scale, 0)
}

move_base_binding = {
    'u': -scale,
    'o': scale
}

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    print("hello there")
    rospy.init_node('move_around')
    initialize_velma()
    err = velma.moveJointImpToCurrentPos()
    rate = rospy.Rate(10)
    try:
        print('ready to get commands')
        while (True):
            # rate.sleep()
            key = get_key()
            print(key)
            if key == 'p':
                break
            elif key in move_head_binding.keys():
                # print('element to pass is {}'.format(move_head_binding[key]))
                q_dest = increment_move(move_head_binding[key])
                velma.moveHead(q_dest, 0.01)
            elif key in move_base_binding.keys():
                last_state = velma.getLastJointState()
                q_map = move_base(move_base_binding[key],10)
                # velma.moveJoint(q_map, 0.5, start_time=0.5, position_tol=15.0 / 180.0 * math.pi)


    except Exception as e:
        print e
    # rospy.init_node('discover_space')
    #
    #
    # initialize_velma()
    # head_tuple = (1, 0)

    # move_head(head_tuple, 1)



















# # !/usr/bin/env python
#
# # Copyright (c) 2011, Willow Garage, Inc.
# # All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions are met:
# #
# #    * Redistributions of source code must retain the above copyright
# #      notice, this list of conditions and the following disclaimer.
# #    * Redistributions in binary form must reproduce the above copyright
# #      notice, this list of conditions and the following disclaimer in the
# #      documentation and/or other materials provided with the distribution.
# #    * Neither the name of the Willow Garage, Inc. nor the names of its
# #      contributors may be used to endorse or promote products derived from
# #       this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
#
# # import roslib
# # roslib.load_manifest('turtlebot_teleop') # No longer needed in catkin!
# import rospy
#
# from geometry_msgs.msg import Twist
#
# import sys, select, termios, tty
#
# msg = """
# Control Your Turtlebot!
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .
# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%
# space key, k : force stop
# anything else : stop smoothly
# CTRL-C to quit
# """
#
# moveBindings = {
#     'i': (1, 0),
#     'o': (1, -1),
#     'j': (0, 1),
#     'l': (0, -1),
#     'u': (1, 1),
#     ',': (-1, 0),
#     '.': (-1, 1),
#     'm': (-1, -1),
# }
#
# speedBindings = {
#     'q': (1.1, 1.1),
#     'z': (.9, .9),
#     'w': (1.1, 1),
#     'x': (.9, 1),
#     'e': (1, 1.1),
#     'c': (1, .9),
# }
#
#
# def getKey():
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ''
#
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key
#
#
# # speed = .2
# speed = 1
# turn = 1
#
#
# def vels(speed, turn):
#     return "currently:\tspeed %s\tturn %s " % (speed, turn)
#
#
# if __name__ == "__main__":
#     settings = termios.tcgetattr(sys.stdin)
#     print("hello there")
#     rospy.init_node('chefbot_teleop')
#     pub = rospy.Publisher('/keyboard/cmd_vel', Twist, queue_size=5)
#
#     x = 0
#     th = 0
#     status = 0
#     count = 0
#     acc = 0.1
#     target_speed = 0
#     target_turn = 0
#     control_speed = 0
#     control_turn = 0
#     try:
#         print msg
#         print vels(speed, turn)
#         while (1):
#             key = getKey()
#             print(key)
#             if key in moveBindings.keys():
#                 x = moveBindings[key][0]
#                 th = moveBindings[key][1]
#                 count = 0
#             elif key in speedBindings.keys():
#                 speed = speed * speedBindings[key][0]
#                 turn = turn * speedBindings[key][1]
#                 count = 0
#
#                 print vels(speed, turn)
#                 if (status == 14):
#                     print msg
#                 status = (status + 1) % 15
#             elif key == ' ' or key == 'k':
#                 x = 0
#                 th = 0
#                 control_speed = 0
#                 control_turn = 0
#             else:
#                 # print('sth else')
#                 count = count + 1
#                 if count > 4:
#                     x = 0
#                     th = 0
#                 if (key == '\x03'):
#                     break
#
#             target_speed = speed * x
#             target_turn = turn * th
#
#             if target_speed > control_speed:
#                 control_speed = min(target_speed, control_speed + 0.02)
#             elif target_speed < control_speed:
#                 control_speed = max(target_speed, control_speed - 0.02)
#             else:
#                 control_speed = target_speed
#
#             if target_turn > control_turn:
#                 control_turn = min(target_turn, control_turn + 0.1)
#             elif target_turn < control_turn:
#                 control_turn = max(target_turn, control_turn - 0.1)
#             else:
#                 control_turn = target_turn
#
#             twist = Twist()
#             twist.linear.x = control_speed;
#             twist.linear.y = 0;
#             twist.linear.z = 0
#             twist.angular.x = 0;
#             twist.angular.y = 0;
#             twist.angular.z = control_turn
#             pub.publish(twist)
#
#             # print("loop: {0}".format(count))
#             # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
#             # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))
#
#     except:
#         print e
#
#     finally:
#         twist = Twist()
#         twist.linear.x = 0;
#         twist.linear.y = 0;
#         twist.linear.z = 0
#         twist.angular.x = 0;
#         twist.angular.y = 0;
#         twist.angular.z = 0
#         pub.publish(twist)
#
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)