#! /usr/bin/env python3

# Import the Python library for ROS
import rospy

import numpy as np
import matplotlib.pyplot as plt

# Import the Odometry message
from nav_msgs.msg import Odometry

# Import the Twist message
from geometry_msgs.msg import Twist

# TF allows to perform transformations between different coordinate frames
import tf

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

from std_msgs.msg import Float64MultiArray

robot_x = -8.7
robot_y = -8.3
robot_yaw = np.pi / 2
k_yaw = 0.5
h_m = 1000
v_max = 0.5
v_min = 0.2


class MoveRobot():

	def __init__(self):
		# Initiate a named node
		rospy.init_node('MoveRobot', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("CTRL + C to stop the turtlebot")

		# What function to call when ctrl + c is issued
		rospy.on_shutdown(self.shutdown)

		rospy.wait_for_service("gazebo/get_model_state")
		self.get_ground_truth = rospy.ServiceProxy(
				"gazebo/get_model_state", GetModelState)

		# Create a Publisher object, will publish on cmd_vel_mux/input/teleop topic
		# to which the robot (real or simulated) is a subscriber
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

		# Creates a var of msg type Twist for velocity
		self.vel = Twist()

		# Set a publish velocity rate of in Hz
		self.rate = rospy.Rate(5)

		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)

		self.target_angle_sub = rospy.Subscriber(
				'/obstacle_detected', Float64MultiArray, self.callback_target)

		self.path_x = []
		self.path_y = []

		self.pos_diffs = []

		self.is_finished = False

		self.linear_v_history = []

		self.current = (robot_x, robot_y, robot_yaw)
		self.set_position(*self.current)

	def set_position(self, x, y, yaw):
		state_msg = ModelState()
		state_msg.model_name = 'turtlebot3_burger'
		state_msg.pose.position.x = x
		state_msg.pose.position.y = y
		state_msg.pose.position.z = 0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		state_msg.pose.orientation.x = quaternion[0]
		state_msg.pose.orientation.y = quaternion[1]
		state_msg.pose.orientation.z = quaternion[2]
		state_msg.pose.orientation.w = quaternion[3]

		rospy.wait_for_service('/gazebo/set_model_state')
		try:
			set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
			resp = set_state(state_msg)
		except rospy.ServiceException as e:
			print(f"Service call failed: {e}")

	def callback_odometry(self, msg):
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
									msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
		self.current = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
		self.path_x.append(msg.pose.pose.position.x)
		self.path_y.append(msg.pose.pose.position.y)

	def callback_target(self, msg):
		target_yaw = msg.data[0]
		h_prime = msg.data[1]
		h_zegond = min(h_prime, h_m)
		v_prime = v_max * (1 - h_zegond / h_m)
		yaw = self.current[2]
		yaw_diff = target_yaw - yaw
		while not -np.pi <= yaw_diff <= np.pi:
			if yaw_diff > np.pi:
				yaw_diff -= 2 * np.pi
			if yaw_diff < -np.pi:
				yaw_diff += 2 * np.pi
		self.vel.angular.z = k_yaw * yaw_diff
		self.vel.linear.x = v_prime * (1 - abs(yaw_diff) / np.pi) + v_min
		# self.vel.linear.x = 0.5
		self.linear_v_history.append(self.vel.linear.x)

	def send_velocity_cmd(self):
		self.vel_pub.publish(self.vel)

	def shutdown(self):
		print("Shutdown!")

		plt.title("Position")
		plt.xlim([-10, 10])
		plt.ylim([-10, 10])
		plt.scatter(self.path_x, self.path_y, s=5)
		plt.savefig('pos.png')
		
		plt.title("Linear V")
		plt.xlim([0, len(self.linear_v_history)])
		plt.ylim([0, 1])
		plt.plot(self.linear_v_history)
		plt.savefig('v.png')

		print(sum(self.linear_v_history) / len(self.linear_v_history))

		# stop TurtleBot
		rospy.loginfo("Stop TurtleBot")

		self.vel.linear.x = 0.0
		self.vel.angular.z = 0.0

		self.vel_pub.publish(self.vel)

		# makes sure robot receives the stop command prior to shutting down
		rospy.sleep(1)

		self.is_finished = True


if __name__ == '__main__':

	try:
		controller = MoveRobot()

		# keeping doing until ctrl+c
		while not rospy.is_shutdown() and not controller.is_finished:

			# send velocity commands to the robots
			controller.send_velocity_cmd()

			# wait for the selected mseconds and publish velocity again
			controller.rate.sleep()

		plt.scatter(controller.path_x, controller.path_y, s=0.5, c='r')
		plt.show()

	except Exception as e:
		rospy.loginfo("move_robot node terminated:", e)
