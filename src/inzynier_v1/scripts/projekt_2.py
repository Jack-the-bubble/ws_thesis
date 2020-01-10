#!/usr/bin/env python

import roslib;

roslib.load_manifest('velma_task_cs_ros_interface')
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
from geometry_msgs.msg import TransformStamped
import thread

if __name__ == "__main__":
	# define some configurations
	q_map_starting = {'torso_0_joint': 0,
	                  'right_arm_0_joint': -0.3, 'left_arm_0_joint': 0.3,
	                  'right_arm_1_joint': -1.8, 'left_arm_1_joint': 1.8,
	                  'right_arm_2_joint': 1.25, 'left_arm_2_joint': -1.25,
	                  'right_arm_3_joint': 0.85, 'left_arm_3_joint': -0.85,
	                  'right_arm_4_joint': 0, 'left_arm_4_joint': 0,
	                  'right_arm_5_joint': -0.5, 'left_arm_5_joint': 0.5,
	                  'right_arm_6_joint': 0, 'left_arm_6_joint': 0}

	q_map_1 = {'torso_0_joint': 0.0,
	           'right_arm_0_joint': -0.3, 'left_arm_0_joint': 0.3,
	           'right_arm_1_joint': -1.57, 'left_arm_1_joint': 1.8,
	           'right_arm_2_joint': 1.57, 'left_arm_2_joint': -1.25,
	           'right_arm_3_joint': 1.57, 'left_arm_3_joint': -0.85,
	           'right_arm_4_joint': 0.0, 'left_arm_4_joint': 0.0,
	           'right_arm_5_joint': -1.57, 'left_arm_5_joint': 0.5,
	           'right_arm_6_joint': 0.0, 'left_arm_6_joint': 0.0}

	rospy.init_node('project_2')

	rospy.sleep(0.5)

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


	def threadFun(new_frame):
		# create circle transform to show the pattern hand has to follow
		# cab_door_tf = velma.getTf('B', 'cab_door')
		# radius = 0.3
		# our_rot = PyKDL.Rotation.RotY(0)
		# # our_vec=PyKDL.Vector(0.6,-0.3,1.3)
		# our_vec = cab_door_tf.p
		# # our_vec=PyKDL.Vector(0.0,0.0,0.0)
		# new_frame = PyKDL.Frame(our_rot, our_vec)
		points = []
		for i in range(8,11):
			points.append(
				PyKDL.Vector(math.cos((2 * math.pi / 10) * i) * radius, math.sin((2 * math.pi / 10) * i) * radius, 0))

		for a in range(3):
			points[a] = new_frame * points[a]

		# print(points)
		br = tf.TransformBroadcaster()


		while True:
			# for i in range(3):
			# 	br.sendTransform((points[i].x(), points[i].y(), points[i].z()),
			# 	                 new_frame.M.GetQuaternion(), rospy.Time.now(), "circle" + str(i), "map")
			br.sendTransform(new_frame.p, new_frame.M.GetQuaternion(), rospy.Time.now(), "new_frame", "map")
			rospy.sleep(0.5)

	def closeHand():
		# closing both hands
		dest_q = [90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, math.pi]
		print "move right:", dest_q
		velma.moveHandRight(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
		velma.moveHandLeft(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
		if velma.waitForHandRight() != 0:
			exitError(8)
		rospy.sleep(0.5)
		if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_q):
			exitError(9)


	def moveRightHand(rotMatrix, point):
		T_B_Trd = PyKDL.Frame(rotMatrix, point)
		if not velma.moveCartImpRight([T_B_Trd], [1.0], None, None, None, None,
		                              PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
			exitError(13)
		if velma.waitForEffectorRight() != 0:
			exitError(14)
		rospy.sleep(0.5)

	def showFrame(frame):
		try:
			thread.start_new_thread(threadFun, (frame,))

		except:
			print ("Could not initialize thread")

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

		rospy.sleep(0.5)
		if not velma.getCoreCsDiag().inStateJntImp():
			exitError(8)


	def countTime(T_B_Trd):
		T_B_Current = velma.getTf("B", "Tr")
		x_diff = T_B_Trd.p[0] - T_B_Current.p[0]
		y_diff = T_B_Trd.p[1] - T_B_Current.p[1]
		z_diff = T_B_Trd.p[2] - T_B_Current.p[2]
		t = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2) + math.pow(z_diff, 2))
		print "TYLE trza przeleciec: ", t
		return t


	def makeWrench(force, torque):
		f = PyKDL.Vector(force[0], force[1], force[2])
		t = PyKDL.Vector(torque[0], torque[1], torque[2])
		return PyKDL.Wrench(f, t)


	def impedMove(T_B_Trd, imp_list, pt, tm = 3):  # cart
		print "Rozpoczecie sterowania impendacyjnego---------------------------"
		# actual_gripper_position = velma.getTf("B", "Tr")  # aktualna pozycja chwytaka
		print "Moving right wrist to pose defined in world frame..."
		my_wrench = makeWrench([5, 5, 5], [5, 5, 5])
		my_twist = PyKDL.Twist(PyKDL.Vector(pt, pt, pt), PyKDL.Vector(pt, pt, pt))
		rospy.sleep(0.5)
		print (len(imp_list))
		"""Tak tutaj dalem 3 zamiast move_time*t bo dla malych ruchow wywalalo errora bo za maly czas wyliczalo"""
		if not velma.moveCartImpRight([T_B_Trd], [tm], None, None, imp_list, [0.5],
		                              max_wrench=my_wrench,
		                              start_time=0.01,
		                              path_tol=my_twist):
			exitError(13)

		if velma.waitForEffectorRight() != 0:  # zglaszane jak chwytak nie moze osiagnac zadanej pozycji
			print("I think it worked?")
			return #actual_gripper_position.p[0], actual_gripper_position.p[1], actual_gripper_position.p[2]
		print "Pose reached, there was no collision"
		return #actual_gripper_position.p[0], actual_gripper_position.p[1], actual_gripper_position.p[2]

	# create cabinet and cabinet door frames
	cab_tf = velma.getTf('B', 'cabinet')
	print("cabinet rot: ", cab_tf.M.GetRPY()[2])
	print(math.pi > cab_tf.M.GetRPY()[2])
	if math.pi/2 > cab_tf.M.GetRPY()[2] and cab_tf.M.GetRPY()[2] > -math.pi/2:
		print("Cannot open the cabinet, it's rotated")
		exitError(42)



	cab_door_tf = velma.getTf('B', 'cab_door')
	radius = 0.4
	# our_rot = PyKDL.Rotation.RotY(math.pi/2)
	our_rot = cab_tf.M*PyKDL.Rotation.RotY(-math.pi/2)*PyKDL.Rotation.RotZ(math.pi)
	# our_vec=PyKDL.Vector(0.6,-0.3,1.3)
	our_vec = cab_door_tf.p
	# our_vec=PyKDL.Vector(0.0,0.0,0.0)
	new_frame = PyKDL.Frame(our_rot, our_vec)



	switchToJimp()
	closeHand()
	tan_angle = math.atan2(cab_tf.p[1], cab_tf.p[0])
	print("rotating torso by: ", tan_angle)
	q_map_1['torso_0_joint'] = tan_angle
	velma.moveJoint(q_map_1, 5.0)
	velma.waitForJoint()
	moveToCimp()

	"""Ustawienie impedancji"""
	lk = 100
	ak = 500
	imped = makeWrench([lk, lk, lk], [ak, ak, ak]),
	# dest_position = PyKDL.Vector(cab_door_tf.p.x()-0.2, cab_door_tf.p.y() + 0.2, cab_door_tf.p.z())
	# dest_position = PyKDL.Vector(cab_tf.p.x()+0.2, cab_door_tf.p.y() + 0.2, cab_tf.p.z()+0.1)
	dest_position = cab_tf*PyKDL.Vector(0.3, 0.2, 0.01)
	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
	# showFrame(T_B_Trd)

	moveRightHand(T_B_Trd.M, T_B_Trd.p)

	# hitting the cabinet
	lk = 100
	ak = 500
	imped = makeWrench([lk, lk, lk], [ak, ak, ak]),
	# dest_position = PyKDL.Vector(cab_door_tf.p.x(), cab_door_tf.p.y() + 0.2, cab_door_tf.p.z())
	# dest_position = PyKDL.Vector(cab_tf.p.x()+0.2, cab_tf.p.y() + 0.15, cab_tf.p.z()+0.01)
	dest_position = cab_tf*PyKDL.Vector(0.1, 0.15, 0.05)
	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
	impedMove(T_B_Trd, imped, 0.1)
	print("Hit the door?")
	# rotate hand a bit and go back a bit

	# grip_tf = velma.getTf("B", "Gr")
	# dest_position = grip_tf * PyKDL.Vector(0, 0, -0.01)
	# T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
	# # showFrame(T_B_Trd)
	# impedMove(T_B_Trd, imped, 0.1)


	print("Go left")
	# dest_position = PyKDL.Vector(cab_door_tf.p.x(), cab_door_tf.p.y() + 0.4, cab_door_tf.p.z())
	# dest_position = PyKDL.Vector(cab_door_tf.p.x()+0.1, cab_tf.p.y(), cab_tf.p.z()+0.01)
	grip_tf = velma.getTf("B", "Gr")
	dest_position = grip_tf*PyKDL.Vector(0, 0.2, -0.02)
	T_B_Trd = PyKDL.Frame(new_frame.M , dest_position)
	
	impedMove(T_B_Trd, imped, 0.1)
	print("grabbed the handle?")
	# move a bit to the rigt so that the hand doesn't touch the other handle

	grip_tf = velma.getTf("B", "Gr")
	dest_position = grip_tf * PyKDL.Vector(0, 0, -0.01)
	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)

	# showFrame(T_B_Trd)
	impedMove(T_B_Trd, imped, 0.1)

	# rotate the hand to make opening easier
	# T_B_Trd = PyKDL.Frame(new_frame.M*PyKDL.Rotation.RotX(-math.pi/6), dest_position)
	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
	new_frame.M = T_B_Trd.M
	impedMove(T_B_Trd, imped, 0.1)


	# lk = 500
	# ak = 500
	lk = 50
	ak = 50
	imped = makeWrench([lk, lk, lk], [ak, ak, ak]),

	# get gripper position
	angle = 0
	grip_tf = velma.getTf("B", "Gr")
	# showFrame(grip_tf)
	y_offset = abs(grip_tf.p.y() - cab_door_tf.p.y())
	x_offset = abs(grip_tf.p.x() - cab_door_tf.p.x())
	radius = math.sqrt(x_offset**2+y_offset**2)
	cab_door_tf = velma.getTf("cabinet", "cab_door")
	# angle = 0#-cab_door_tf.M.GetRPY()[2]
	# old_angle=0
	# while angle < ((math.pi/2)-0.1):
	# 	grip_tf = velma.getTf("B", "Gr")
	# 	cab_door_tf_old = cab_door_tf
	# 	cab_door_tf = velma.getTf("cabinet", "cab_door")

	# 	# dest_position = grip_tf*PyKDL.Vector(0, 0,-0.05)
	# 	dest_position = grip_tf*PyKDL.Vector(0, 0,-0.5)

	# 	needed_rot = cab_door_tf.M.GetRPY()[2] - cab_door_tf_old.M.GetRPY()[2]
		

	# 	 # new_frame = new_frame*PyKDL.Frame(PyKDL.Rotation.RotX(-needed_rot))
	# 	new_frame = new_frame*PyKDL.Frame(PyKDL.Rotation.RotX(-math.pi/2))
		

	# 	# next_frame = PyKDL.Frame(new_frame.M*PyKDL.Rotation.RotX(math.pi/10), dest_position)
	# 	# T_B_Trd = PyKDL.Frame(next_frame.M, dest_position)
	# 	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
	# 	# showFrame(T_B_Trd)
	# 	impedMove(T_B_Trd, imped, 0.3, tm = 6)
	# 	angle = cab_door_tf.M.GetRPY()[2]

	# 	if abs(old_angle-angle)<0.01:
	# 		print("Error cabinet door slipped")
	# 		break

	# 	old_angle=angle
	# 	print("Finished movement - success I guess")
	# 	# moveRightHand(new_frame.M*PyKDL.Rotation.RotY(math.pi/2), cab_door_tf.p)


	# dest_position = grip_tf*PyKDL.Vector(0, 0,-0.05)
	dest_position = grip_tf*PyKDL.Vector(0, 0,-0.5)

	# needed_rot = cab_door_tf.M.GetRPY()[2] - cab_door_tf_old.M.GetRPY()[2]
		

		 # new_frame = new_frame*PyKDL.Frame(PyKDL.Rotation.RotX(-needed_rot))
	new_frame = new_frame*PyKDL.Frame(PyKDL.Rotation.RotX(-math.pi/4))
		

		# next_frame = PyKDL.Frame(new_frame.M*PyKDL.Rotation.RotX(math.pi/10), dest_position)
		# T_B_Trd = PyKDL.Frame(next_frame.M, dest_position)
	T_B_Trd = PyKDL.Frame(new_frame.M, dest_position)
		# showFrame(T_B_Trd)
	impedMove(T_B_Trd, imped, 0.5, tm = 6)
		# angle = cab_door_tf.M.GetRPY()[2]

	# lk = 100
	# ak = 100
	# imped = makeWrench([lk, lk, lk], [ak, ak, ak]),

	grip_tf = velma.getTf("B", "Gr")
	dest_position = grip_tf*PyKDL.Vector(0, 0,-0.5)
	grip_tf = grip_tf*PyKDL.Frame(PyKDL.Rotation.RotX(-math.pi/4))
	T_B_Trd = PyKDL.Frame(grip_tf.M, dest_position)
		# showFrame(T_B_Trd)
	impedMove(T_B_Trd, imped, 0.5, tm = 6)

	# lk = 200
	# ak = 200
	# imped = makeWrench([lk, lk, lk], [ak, ak, ak]),

	# grip_tf = velma.getTf("B", "Gr")
	# dest_position = grip_tf*PyKDL.Vector(0, 0,-0.05)
	# T_B_Trd = PyKDL.Frame(grip_tf.M, dest_position)
	# 	# showFrame(T_B_Trd)
	# # impedMove(T_B_Trd, imped, 0.1, tm = 6)



	# grip_tf = velma.getTf("B", "Gr")
	# dest_position = grip_tf*PyKDL.Vector(0, 0,0.01)
	# grip_tf = grip_tf*PyKDL.Frame(PyKDL.Rotation.RotX(-math.pi/4))
	# T_B_Trd = PyKDL.Frame(grip_tf.M, dest_position)
	# 	# showFrame(T_B_Trd)
	# impedMove(T_B_Trd, imped, 0.5, tm = 6)

	


	print ("Opened door")



	grip_tf = velma.getTf("B", "Gr")
	new_frame = grip_tf
	moveRightHand(new_frame.M, new_frame.p)
	impedMove(new_frame, imped, 0.1)

	lk = 500
	ak = 500
	imped = makeWrench([lk, lk, lk], [ak, ak, ak]),

	
	grip_tf = velma.getTf("B", "Gr")
	new_frame = grip_tf*PyKDL.Frame(PyKDL.Vector(0, -0.1, 0))

	impedMove(new_frame, imped, 0.1)
	lk = 500
	ak = 500
	imped = makeWrench([lk, lk, lk], [ak, ak, ak]),

	print ("Opened ")
	new_frame = grip_tf*PyKDL.Frame(PyKDL.Vector(0,0.08, -0.18))
	impedMove(new_frame, imped, 0.1)
	switchToJimp()
	q_map_1['torso_0_joint'] = 0
	velma.moveJoint(q_map_1, 5.0)

