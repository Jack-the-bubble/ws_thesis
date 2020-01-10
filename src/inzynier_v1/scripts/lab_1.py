#!/usr/bin/env python

## Runs test for tool motions in cart_imp mode.
# @ingroup integration_tests
# @file test_cimp_tool.py
# @namespace scripts.test_cimp_tool Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author:
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.57,
        'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.57,
        'right_arm_3_joint':1.57,   'left_arm_3_joint':-1.7,
        'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,  'left_arm_5_joint':1.57,
        'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }

    rospy.init_node('test_cimp_tool')

    rospy.sleep(0.5)

    print "This test/tutorial executes simple tool motions"\
        " in Cartesian impedance mode.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    if velma.enableMotors() != 0:
        exitError(2)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(3)

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(4)
    print "Planner init ok"

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(8)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(9)

    # get initial configuration
    js_init = velma.getLastJointState()


    


    

    planAndExecute(q_map_1)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(12)

    print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
    print "At every state switch to cart_imp, the tool frames are reset"


    # creating new frame

    radius = 0.1
    our_rot = PyKDL.Rotation.RotY((math.pi/4))
    our_vec=PyKDL.Vector(0.6,-0.3,1.3)
    # our_vec=PyKDL.Vector(0.0,0.0,0.0)
    new_frame = PyKDL.Frame(our_rot, our_vec)
    points = []
    for i in range(11):
        points.append(PyKDL.Vector(math.cos((2*math.pi/10)*i)*radius, math.sin((2*math.pi/10)*i)*radius, 0))

    for a in range(0, 11):
         points[a] = new_frame*points[a]

    # print(points)
    # br = tf.TransformBroadcaster()

    # while True:
    #     for i in range(10):
    #         br.sendTransform((points[i].x(), points[i].y(), points[i].z()), new_frame.M.GetQuaternion(), rospy.Time.now(), "circle"+str(i), "map")
    #         # br.sendTransform((points[1].x(), points[1].y(), points[1].z()), new_frame.M.GetQuaternion(), rospy.Time.now(), "circle1", "map")
    #         # br.sendTransform((points[2].x(), points[2].y(), points[2].z()), new_frame.M.GetQuaternion(), rospy.Time.now(), "circle2", "map")
    #     rospy.sleep(0.1)


    for i in range(11):
        T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), points[i])
        if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(13)
        if velma.waitForEffectorRight() != 0:
            exitError(14)
        rospy.sleep(0.5)
        print "Calculating difference between desiread and reached pose..."
        T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
        print T_B_T_diff
        if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
            exitError(15)

    # T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), points[0])
    # velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)

    # print "Moving right wrist to pose defined in world frame..."
    # T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    # if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(13)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(14)
    # rospy.sleep(0.5)
    # print "Calculating difference between desiread and reached pose..."
    # T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
    # print T_B_T_diff
    # if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
    #     exitError(15)

    # print "Rotating right writs by 45 deg around local z axis (right-hand side matrix multiplication)"
    # T_B_Tr = velma.getTf("B", "Tr")
    # T_B_Trd = T_B_Tr * PyKDL.Frame(PyKDL.Rotation.RotZ(45.0/180.0*math.pi))
    # if not velma.moveCartImpRight([T_B_Trd, T_B_Tr], [2.0, 4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(16)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(17)
    # rospy.sleep(0.5)

    # print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
    # T_B_Wr = velma.getTf("B", "Wr")
    # T_Wr_Gr = velma.getTf("Wr", "Gr")
    # if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(18)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(19)

    # print "the right tool is now in 'grip' pose"
    # rospy.sleep(0.5)

    # print "Rotating right equilibrium pose by -45 deg around local x axis (right-hand side matrix multiplication)"
    # T_B_Tr = velma.getTf("B", "Tr")
    # T_B_Trd = T_B_Tr * PyKDL.Frame(PyKDL.Rotation.RotX(-45.0/180.0*math.pi))
    # if not velma.moveCartImpRight([T_B_Trd, T_B_Tr], [2.0, 4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(20)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(21)
    # rospy.sleep(0.5)

    planAndExecute(q_map_starting)

    exitError(0)

