#! /usr/bin/env python3

# Import the Python library for ROS
import rospy

import numpy as np
import math

# TF allows to perform transformations between different coordinate frames
import tf

# Import the Odometry message
from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan
# from kobuki_msgs.msg import BumperEvent

from std_msgs.msg import Float64MultiArray


heading_coef = 1
previous_coef = 1
path_coef = 5

valley_threshold = 65000
min_angle = 1
cell_size = 0.1
map_size = round((20 + 5) / cell_size)
number_of_sectors = 72
alpha = (2 * np.pi) / number_of_sectors
ws = 7 / cell_size
a = ((ws - 1) / 2) * math.sqrt(2)
b = 1

d_star = 1
path = [[-6.5, 4], [3.5, 4.5], [4.5, 4.5], [4.5, 3.5], [3, 1.5],
				[2, 0], [-3, -3], [9.5, -5.5], [9.5, -6], [9.5, -7], [9, -8], [0, -9], [-8, -9]]


class ObstacleDetector():

	def __init__(self):
		# Initiate a named node
		rospy.init_node('obstacle', anonymous=False)

		# Set a publish velocity rate of in Hz
		self.rate = rospy.Rate(5)

		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)

		self.laser = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
		# self.bumper = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.callback_bumper)

		self.target_angle_pub = rospy.Publisher(
				'/obstacle_detected', Float64MultiArray, queue_size=1)
		self.target_angle = Float64MultiArray()

		self.is_finished = False

		self.current = None
		self.previous_angle = None
		self.targt_index = 0

		self.use_path = 1
		self.need_map = True

		self.obstacle_map = np.zeros((map_size, map_size))

	def callback_laser(self, msg):
		if not self.current:
			return

		robot_x, robot_y, robot_yaw = self.current

		range_len = len(msg.ranges)
		for i in range(range_len):
			if msg.ranges[i] == math.inf:
				continue
			angle = robot_yaw + msg.angle_min + i * msg.angle_max / range_len
			x, y = self.scale_to_map(robot_x + np.cos(angle) * msg.ranges[i],
															 robot_y + np.sin(angle) * msg.ranges[i])
			self.obstacle_map[x, y] += msg.ranges[i]

		sectors = np.zeros(number_of_sectors)
		map_x, map_y = self.scale_to_map(robot_x, robot_y)

		if self.need_map:
			self.print_map(map_x, map_y)

		for x in range(round(map_x - ((ws - 1) / 2)), round(map_x + ((ws - 1) / 2))):
			for y in range(round(map_y - ((ws - 1) / 2)), round(map_y + ((ws - 1) / 2))):
				k = math.floor(math.atan2(y - map_y, x - map_x) / alpha)
				d = math.sqrt((x - map_x) ** 2 + (y - map_y) ** 2)
				sectors[k] += (self.obstacle_map[x, y] ** 2) * (a - (b * d))
		# print(sectors)
		# print(min(sectors), np.average(sectors), max(sectors))

		valley = []
		begin = 0
		for i in range(number_of_sectors):
			if sectors[i] > valley_threshold:
				if i - begin > 0:
					valley.append([begin, i])
				begin = i + 1
			elif i == number_of_sectors - 1:
				if range_len - begin > 0:
					if valley and valley[0][0] == 0:
						valley[0][0] = begin
					else:
						valley.append([begin, 0])

		i = 0
		while i < len(valley):
			size = valley[i][1] - valley[i][0]
			if size <= 0:
				size = number_of_sectors - size

			if size < min_angle:
				valley.pop(i)
			else:
				i += 1

		# for v in valley:
		# 	p = [v[0] * 5, v[1] * 5]
		# 	print(p, end=" ")
		# 	if v[1] < v[0]:
		# 		print(max(list(sectors[:v[1]]) + list(sectors[v[0]:])), end=", ")
		# 	elif v[0] != v[1]:
		# 		print(max(sectors[v[0]:v[1]]), end=", ")
		# print()

		final_angle = None
		if not valley or valley[0] == [0, 0]:
			final_angle = robot_yaw
		else:
			angle_choices = []
			for v in valley:
				angle_mean = (v[0] + v[1]) * alpha / 2
				if v[1] <= v[0]:
					angle_mean += np.pi
				angle_choices.append(angle_mean)

			for choice in angle_choices:
				if not final_angle or self.angle_cost(choice, robot_yaw) < self.angle_cost(final_angle, robot_yaw):
					final_angle = choice
		self.previous_angle = final_angle
		# print("target angle:", round(final_angle / np.pi * 180, 2),
		# " heading: ", round(robot_yaw / np.pi * 180, 2))
		self.target_angle.data = [final_angle,
															sectors[math.floor(robot_yaw / alpha)]]
		self.target_angle_pub.publish(self.target_angle)

	def print_map(self, map_x, map_y):
		for x in range(round(map_x - ((ws - 1) / 2)), round(map_x + ((ws - 1) / 2))):
			for y in range(round(map_y - ((ws - 1) / 2)), round(map_y + ((ws - 1) / 2))):
				if x == map_x and y == map_y:
					print("O", end="")
				else:
					if self.obstacle_map[x, y] < 10:
						print(" ", end="")
					else:
						print(int(math.log10(self.obstacle_map[x, y])), end="")
			print()

	def scale_to_map(self, x, y):
		return math.floor(x / cell_size), math.floor(y / cell_size)

	def angle_cost(self, angle, heading):
		target = path[self.targt_index]
		target_yaw = math.atan2(
				target[1] - self.current[1], target[0] - self.current[0])
		if self.previous_angle:
			return ((heading_coef * abs(self.angle_diff(heading, angle))) +
							(previous_coef * abs(self.angle_diff(self.previous_angle, angle))) +
							(self.use_path * path_coef * abs(self.angle_diff(target_yaw, angle))))
		return ((heading_coef * abs(self.angle_diff(heading, angle))) +
						(self.use_path * path_coef * abs(self.angle_diff(target_yaw, angle))))

	def angle_diff(self, angle1, angle2):
		diff = angle1 - angle2
		while not -np.pi <= diff <= np.pi:
			if diff > np.pi:
				diff -= 2 * np.pi
			if diff < -np.pi:
				diff += 2 * np.pi
		return diff

#	def callback_bumper(self, msg):
#		print(msg)

	def callback_odometry(self, msg):
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
									msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
		self.current = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

		if self.use_path:
			target = path[self.targt_index]
			if math.sqrt((self.current[0] - target[0]) ** 2 + (self.current[1] - target[1]) ** 2) < d_star:
				self.targt_index = (self.targt_index + 1) % len(path)
				# print("next target is: ", path[self.targt_index])


if __name__ == '__main__':
	obstacle_detector = ObstacleDetector()
	while not obstacle_detector.is_finished:
		obstacle_detector.rate.sleep()
