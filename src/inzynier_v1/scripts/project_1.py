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
from docutils.utils.math.latex2mathml import mover

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
from geometry_msgs.msg import TransformStamped






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
		'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.8,
		'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.25,
		'right_arm_3_joint':1.57,   'left_arm_3_joint':-0.85,
		'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
		'right_arm_5_joint':-1.57,  'left_arm_5_joint':0.5,
		'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }

	rospy.init_node('test_cimp_tool')

	rospy.sleep(0.5)

	# print "This test/tutorial executes simple tool motions"\
	#     " in Cartesian impedance mode.\n"

	# print "Running python interface for Velma..."
	velma = VelmaInterface()
	# print "Waiting for VelmaInterface initialization..."
	if not velma.waitForInit(timeout_s=10.0):
		print "Could not initialize VelmaInterface\n"
		exitError(1)
	# print "Initialization ok!\n"

	if velma.enableMotors() != 0:
		exitError(2)

	diag = velma.getCoreCsDiag()
	if not diag.motorsReady():
		print "Motors must be homed and ready to use for this test."
		exitError(3)

	# print "waiting for Planner init..."
	p = Planner(velma.maxJointTrajLen())
	if not p.waitForInit():
		print "could not initialize PLanner"
		exitError(4)
	# print "Planner init ok"

	# define octomap object
	oml = OctomapListener('/octomap_binary')
	rospy.sleep(1.0)
	octomap = oml.getOctomap(timeout_s=5.0)
	p.processWorld(octomap)

 


	def closeHand():
			# closing right hand????
		dest_q = [90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, math.pi]
		print "move right:", dest_q
		velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
		velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
		if velma.waitForHandRight() != 0:
			exitError(8)
		rospy.sleep(0.5)
		if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
			exitError(9)

	def openHand():
		dest_q = [0,0,0,0]
		print "move right:", dest_q
		velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
		if velma.waitForHandRight() != 0:
			exitError(8)
		rospy.sleep(0.5)
		if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
			exitError(9)

	def closeFingers():
		dest_q = [90.0/180.0*math.pi,90.0/180.0*math.pi,90.0/180.0*math.pi,0]
		print "move right:", dest_q
		velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
		if velma.waitForHandRight() != 0:
			exitError(8)
		rospy.sleep(0.5)
		return dest_q
		# if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		#     exitError(9)

	def moveRightHand(rotMatrix, point):
		T_B_Trd = PyKDL.Frame(rotMatrix, point)
		if not velma.moveCartImpRight([T_B_Trd], [1.0], None, None, None, None,
									  PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
			exitError(13)
		if velma.waitForEffectorRight() != 0:
			exitError(14)
		rospy.sleep(0.5)

	def findJar():
		# get rot and vec from jar

		T_B_Jar = velma.getTf('B', 'jar')

		# get rotation so the gripper doesn't twist
		jar_roll, _, jar_yaw = T_B_Jar.M.GetRPY()

		jar_rot = PyKDL.Rotation.RotY(math.pi / 2)

		# check if jar is upside down
		if math.pi - 0.1 < jar_roll < math.pi + 0.1 or -math.pi + 0.1 > jar_roll > -math.pi - 0.1:
			prep_rot = PyKDL.Rotation.RotX(-jar_roll)
		else:
			prep_rot = PyKDL.Rotation.RotX(0)

		prep_rot = prep_rot * PyKDL.Rotation.RotZ(-jar_yaw)
		prep_rot = prep_rot * jar_rot
		prep_frame = PyKDL.Frame(prep_rot, PyKDL.Vector())

		glob_prep_frame = T_B_Jar * prep_frame
		jar_point_prep = glob_prep_frame * PyKDL.Vector(-0.1, 0, -0.2)
		return glob_prep_frame, jar_point_prep

	def moveToCimp():
		# moving tool frame to grip?
		print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
		T_B_Wr = velma.getTf("B", "Wr")
		T_Wr_Gr = velma.getTf("Wr", "Gr")
		if not velma.moveCartImpRight([T_B_Wr * T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None,
									  PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
			exitError(18)
		if velma.waitForEffectorRight() != 0:
			exitError(19)
		print "the right tool is now in 'grip' pose"
		rospy.sleep(0.5)

		print "Switch to cart_imp mode (no trajectory)..."
		if not velma.moveCartImpRightCurrentPos(start_time=0.2):
			exitError(10)
		if velma.waitForEffectorRight() != 0:
			exitError(11)
		rospy.sleep(0.5)

	def switchToJimp():
		print "Switch to jimp_imp mode (no trajectory)..."
		if not velma.moveJointImpToCurrentPos(start_time=0.2):
			exitError(10)
			rospy.sleep(0.5)
		# if velma.waitForEffectorRight() != 0:
		# 	exitError(11)

	# define a function for frequently used routine in this test
	def planAndExecute(q_dest, pos_tol = 5.0, vel_tol = 5.0):
		print "Planning motion to the goal position using set of all joints..."
		print "Moving to valid position, using planned trajectory."
		goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
		for i in range(5):
			rospy.sleep(0.5)
			js = velma.getLastJointState()
			print "Planning (try", i, ")..."
			traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.1, planner_id="RRTConnect")
			if traj == None:
				continue
			print "Executing trajectory..."
			if not velma.moveJointTraj(traj, start_time=0.5, position_tol = pos_tol/180.0*math.pi, velocity_tol = vel_tol/180.0*math.pi ):
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

	########
	#################################### prepare necessary frames
	######## frame to prepare to grab the jar
	glob_prep_frame, jar_point_prep = findJar()
	jar_angle = math.atan2(jar_point_prep.y(), jar_point_prep.x())



	######## point to grab the jar
	jar_vec_grab = PyKDL.Vector(0.0, 0.0, 0.00)
	jar_point_grab = glob_prep_frame*jar_vec_grab

	######## point to lift the jar
	jar_vec_up = PyKDL.Vector(-0.4, 0.0, 0.0)
	jar_point_up = glob_prep_frame * jar_vec_up

	######## go to the second table
	# get rot and vec from jar
	T_B_Tab = velma.getTf('B', 'tab_0')
	# get rotation so the gripper doesn't twist


	jar_roll, _, jar_yaw = T_B_Tab.M.GetRPY()
	tab_vec = T_B_Tab.p
	angle = math.atan2(tab_vec.y(), tab_vec.x())


	jar_rot_z = PyKDL.Rotation.RotZ(-jar_yaw+angle)
	# jar_rot_x = PyKDL.Rotation.RotX(math.pi / 2)
	# tab_rot = T_B_Tab.M * jar_rot_x * jar_rot_z
	jar_rot_y = PyKDL.Rotation.RotY(math.pi / 2)
	tab_rot = T_B_Tab.M * jar_rot_z * jar_rot_y
	tab_point = PyKDL.Vector(-0.55, 0.2, 1.25)
	# put point in global frame
	tab_point = T_B_Tab * tab_point
	dist = math.sqrt(tab_point.x()**2+tab_point.y()**2)
	print(dist)
	if dist > 1.3:
		exitError(42)

	######## go back a bit so the gripper doesn't touch the jar when going back
	tab_vec_up = PyKDL.Vector(-0.55, 0.2, 1.35)
	tab_point_up = T_B_Tab * tab_vec_up

	###############
	############### start moving
	###############

	closeHand()

	# switchToJimp()
	# _, q_map_2 = velma.getLastJointState()
	# print(q_map_2)
	# q_map_2['torso_0_joint'] = jar_angle
	# planAndExecute(q_map_2, pos_tol = 200.0, vel_tol = 20.0)
	# moveToCimp()

	q_map_1['torso_0_joint'] = jar_angle

	velma.moveJoint(q_map_1, 5.0)
	velma.waitForJoint()

	# planAndExecute(q_map_1)


	moveToCimp()


	diag = velma.getCoreCsDiag()
	if not diag.inStateCartImp():
		print "The core_cs should be in cart_imp state, but it is not"
		exitError(12)

	print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
	print "At every state switch to cart_imp, the tool frames are reset"

	

	jar_taken=0
	
	while not jar_taken :

		glob_prep_frame, jar_point_prep = findJar()
		

		jar_angle = math.atan2(jar_point_prep.y(), jar_point_prep.x())

		jar_vec_grab = PyKDL.Vector(0.0, 0.0, 0.00)
		jar_point_grab = glob_prep_frame*jar_vec_grab


		moveRightHand(glob_prep_frame.M, jar_point_prep)
		openHand()

		# go to grab the jar
		moveRightHand(glob_prep_frame.M, jar_point_grab)
		dest_q = closeFingers()

		# go up
		moveRightHand(glob_prep_frame.M, jar_point_up)
		if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
			jar_taken=1



	# br = tf.TransformBroadcaster()
	# while True:
	# 	br.sendTransform((tab_point.x(), tab_point.y(), tab_point.z()), tab_rot.GetQuaternion(), rospy.Time.now(),
 #                       "jar_rot", "map")
	# 	x = input()
	# 	if x == 1:
	# 		break

	# change to joint mode and turn the torso to put the jar to the other table
	switchToJimp()

	_, q_map_2 = velma.getLastJointState()
	q_map_2['torso_0_joint'] = angle
	planAndExecute(q_map_2, pos_tol=20.0, vel_tol=20.0)

	moveToCimp()
	

	moveRightHand(tab_rot, tab_point)
	openHand()
	moveRightHand(tab_rot, tab_point_up)
	_, q_map_2 = velma.getLastJointState()
	q_map_2['torso_0_joint'] = 0
 	print "Switch to jimp_imp mode (no trajectory)..."
	if not velma.moveJointImpToCurrentPos(start_time=0.2):
		exitError(10)
	if velma.waitForEffectorRight() != 0:
		exitError(11)

 	planAndExecute(q_map_2, pos_tol=20.0, vel_tol=20.0)

 	moveToCimp()

	moveRightHand(glob_prep_frame.M, jar_point_prep)


	switchToJimp()
	print "Switch to jnt_imp mode (no trajectory)..."
	velma.moveJointImpToCurrentPos(start_time=0.2)
	error = velma.waitForJoint()
	if error != 0:
		print "The action should have ended without error, but the error code is", error
		exitError(7)

	planAndExecute(q_map_starting)

	exitError(0)

