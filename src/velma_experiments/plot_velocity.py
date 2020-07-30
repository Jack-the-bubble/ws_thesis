
from matplotlib import pyplot as plt
import numpy as np
import rospy


from geometry_msgs.msg import Twist

class PlotMaker:
	def __init__(self):
		self.sub = rospy.Subscriber('cmd_vel', Twist, self.vel_callback)
		self.max_size = 5000
		self.vel_array = []


		plt.title('test plot')
		plt.xlabel('some number')
		plt.ylabel('some value')

	def vel_callback(self, msg):
		if len(self.vel_array) > self.max_size:
			self.vel_array.pop(0)

		self.vel_array.append(msg.linear.x)

	def update(self):
		x_axis = [i for i in range(1, len(self.vel_array) + 1)]
		plt.plot(x_axis, self.vel_array)
		plt.show()


if __name__ == '__main__':
	rospy.init_node('plot_velocity')
	plot_maker = PlotMaker()

	while not rospy.is_shutdown():
		plot_maker.update()
		rospy.sleep(1)

	rospy.spin()