#!/usr/bin/env python

import PyKDL
import rospy
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import *
from geometry_msgs.msg import TransformStamped, Twist


def makeWrench(force, torque):
	f = PyKDL.Vector(force[0], force[1], force[2])
	t = PyKDL.Vector(torque[0], torque[1], torque[2])
	return PyKDL.Wrench(f, t)


class VelmaController:
	def __init__(self):
		self.current_velocity = Twist()
		self.time_stamp = rospy.Time.now()
		self.base_ongoing = False

		self.velma = VelmaInterface()

		# print "Waiting for VelmaInterface initialization..."
		if not self.velma.waitForInit(timeout_s=10.0):
			print "Could not initialize VelmaInterface\n"
			exitError(1)
		# print "Initialization ok!\n"

		if self.velma.enableMotors() != 0:
			exitError(2)

		diag = self.velma.getCoreCsDiag()
		if not diag.motorsReady():
			print "Motors must be homed and ready to use for this test."
			exitError(3)

		velma_cmd = rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)

	def main_loop(self):
		while not rospy.is_shutdown() and self.base_ongoing:
			if self.is_moving():
				if not self.velma.getCoreCsDiag().inStateJntImp():
					self.switchToJimp()
				print ("Implement turning into direction of travel")
				desired_angle = math.atan2(self.current_velocity.linear.y, self.current_velocity.linear.x)
				print("desired_angle: {}".format(desired_angle))
				q_map_starting['torso_0_joint'] = desired_angle #q_map_starting['torso_0_joint'] + 0.1 #desired_angle
				self.velma.moveJoint(q_map_starting, 10.0)
				# q_dest_head = (-0.2, desired_angle)
				# self.velma.moveHead(q_dest_head, 1.0, start_time=0.5)
				error = self.velma.waitForJoint()
				if error != 0:
					print ("error while waiting for robot to turn")
					exitError(1)

				print("finished move")



				rospy.sleep(0.5)

			else:
				print ("Not moving, nothing to do in this node")

	def is_moving(self):
		current_time = rospy.Time.now()
		print ("Current time: {}\n last recorded: {}".format(current_time, self.time_stamp))
		print("X velocity: {}\n Y velocity: {}".format(self.current_velocity.linear.x, self.current_velocity.linear.y))
		if current_time.secs - self.time_stamp.secs < 2 and (self.current_velocity.linear.x != 0 or self.current_velocity.linear.y != 0):
			print ("on the move")
			return True
		else:
			self.base_ongoing = False
			print ("not moving")
			return False


	def closeHand(self):
		# closing both hands
		dest_q = [90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, math.pi]
		print "move right:", dest_q
		self.velma.moveHandRight(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
		self.velma.moveHandLeft(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
		if self.velma.waitForHandRight() != 0:
			exitError(8)
		rospy.sleep(0.5)
		if not isHandConfigurationClose(self.velma.getHandRightCurrentConfiguration(), dest_q):
			exitError(9)

	def impedMove(self, T_B_Trd, imp_list, pt, tm=3):  # cart
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

		if self.velma.waitForEffectorRight() != 0:  # zglaszane jak chwytak nie moze osiagnac zadanej pozycji
			print("I think it worked?")
			return  # actual_gripper_position.p[0], actual_gripper_position.p[1], actual_gripper_position.p[2]
		print "Pose reached, there was no collision"
		return  # actual_gripper_position.p[0], actual_gripper_position.p[1], actual_gripper_position.p[2]

	def moveToCimp(self):
		# moving tool frame to grip?
		print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
		T_B_Wr = self.velma.getTf("B", "Wr")
		T_Wr_Gr = self.velma.getTf("Wr", "Gr")
		if not self.velma.moveCartImpRight([T_B_Wr * T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None,
		                              PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
			exitError(18)
		if self.velma.waitForEffectorRight() != 0:
			exitError(19)
		print "the right tool is now in 'grip' pose"
		rospy.sleep(0.5)

		print "Switch to cart_imp mode (no trajectory)..."
		if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
			exitError(10)
		if self.velma.waitForEffectorRight() != 0:
			exitError(11)
		rospy.sleep(0.5)

	def switchToJimp(self):
		print "Switch to jimp_imp mode (no trajectory)..."
		if not self.velma.moveJointImpToCurrentPos(start_time=0.2):
			exitError(10)
			rospy.sleep(0.5)

		rospy.sleep(0.5)
		if not self.velma.getCoreCsDiag().inStateJntImp():
			exitError(8)

	def velocity_callback(self, msg):
		self.current_velocity = msg
		self.time_stamp = rospy.Time.now()
		self.base_ongoing = True
		self.main_loop()


if __name__ == '__main__':
	# first configuration
	q_map_starting = {'torso_0_joint': 0,
	                  'right_arm_0_joint': -0.3, 'left_arm_0_joint': 0.3,
	                  'right_arm_1_joint': -1.8, 'left_arm_1_joint': 1.8,
	                  'right_arm_2_joint': 1.25, 'left_arm_2_joint': -1.25,
	                  'right_arm_3_joint': 0.85, 'left_arm_3_joint': -0.85,
	                  'right_arm_4_joint': 0, 'left_arm_4_joint': 0,
	                  'right_arm_5_joint': -0.5, 'left_arm_5_joint': 0.5,
	                  'right_arm_6_joint': 0, 'left_arm_6_joint': 0}

	rospy.init_node('follow_base_with_head')
	rospy.sleep(0.5)



	velma_con = VelmaController()
	velma_con.switchToJimp()
	velma_con.closeHand()

	# rospy.INFO('finished initializing body, starting to move')
	print('finished initializing body, starting to move')

	velma_con.main_loop()

	# q_map_starting['torso_0_joint'] = 0.6
	# velma_con.velma.moveJoint(q_map_starting, 5.0)

	# rospy.INFO('move finished')
	print ('move finished')

	rospy.spin()



