#! /usr/bin/env python3

# Import the Python library for ROS
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import math

# TF allows to perform transformations between different coordinate frames
import tf

# Import the Odometry message
from nav_msgs.msg import Odometry

#
from sensor_msgs.msg import LaserScan
# from kobuki_msgs.msg import BumperEvent

from std_msgs.msg import Float64


heading_coef = 1
previous_coef = 1
valley_threshold = 10000
min_angle = 2
cell_size = 0.1
map_size = round((20+5)/cell_size)
number_of_sectors = 72
alpha = 2*np.pi/number_of_sectors
ws = 7 / cell_size
a = (ws-1)/2*math.sqrt(2)
b = 1


class ObstacleDetector():

	def __init__(self):
		# Initiate a named node
		rospy.init_node('obstacle', anonymous=False)
		
		# rospy.wait_for_service("gazebo/get_model_state")
		# self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
				
		# Set a publish velocity rate of in Hz
		self.rate = rospy.Rate(5)
		
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)

		self.laser = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
		# self.dumper = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.callback_bumper)
		
		self.target_angle_pub = rospy.Publisher('/target', Float64, queue_size=1)
		self.target_angle = Float64()
		
		self.is_finished = False
		
		self.current = None
		self.previous_angle = None

		self.obstacle_map = np.zeros((map_size, map_size))

	def callback_laser(self, msg):
		if not self.current:
			return
		
		robot_x, robot_y, robot_yaw = self.current
		
		range_len = len(msg.ranges)
		for i in range(range_len):
			if msg.ranges[i] == math.inf:
				continue
			angle = robot_yaw+msg.angle_min+i*msg.angle_max/range_len
			x, y = self.scale_to_map(robot_x + np.cos(angle)*msg.ranges[i],
								robot_y + np.sin(angle)*msg.ranges[i])
			self.obstacle_map[x, y] += 1
		
		sectors = np.zeros(number_of_sectors)
		map_x, map_y = self.scale_to_map(robot_x, robot_y)
		
		""" for x in range(round(map_x + (1-ws)/2), round(map_x + (ws-1)/2)):
			for y in range(round(map_y + (1-ws)/2), round(map_y + (ws-1)/2)):
				if self.obstacle_map[x, y] < 10:
					print(0, end=' ')
				else:
					print(int(math.log10(self.obstacle_map[x, y])), end=' ')
			print() """
		
		for x in range(round(map_x + (1-ws)/2), round(map_x + (ws-1)/2)):
			for y in range(round(map_y + (1-ws)/2), round(map_y + (ws-1)/2)):
				k = math.floor(math.atan2(y-map_y, x-map_x)/alpha)
				d = math.sqrt((x - map_x)**2 + (y - map_y)**2)
				sectors[k] += self.obstacle_map[x, y]**2 * (a-b*d)
		print(sectors)
		
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
		final_angle = None
		if valley[0] == [0, 0]:
			final_angle = robot_yaw
		else:
			i = 0
			while i < len(valley):
				size = valley[i][1] - valley[i][0]
				if size <= 0:
					size = number_of_sectors - size
				if size < min_angle:
					valley.pop(i)
				else:
					i += 1
			angle_choices = []
			for v in valley:
				angle_mean = (v[0]+v[1])*alpha/2
				if v[1] <= v[0]:
					angle_mean = angle_mean + np.pi
				angle_choices.append(angle_mean)
			for choice in angle_choices:
				if not final_angle or self.angle_cost(choice, robot_yaw) < self.angle_cost(final_angle, robot_yaw):
					final_angle = choice
		self.previous_angle = final_angle
		print("target angle:", final_angle)
		self.target_angle.data = final_angle
		self.target_angle_pub.publish(self.target_angle)
		
		""" valley = []
		begin = 0
		range_len = len(msg.ranges)
		for i in range(range_len):
			if msg.ranges[i] < valley_threshold:
				if i - begin > 0:
					valley.append([begin, i])
				begin = i + 1
			elif i == range_len - 1:
				if range_len - begin > 0:
					if valley and valley[0][0] == 0:
						valley[0][0] = begin
					else:
						valley.append([begin, 0])
		i = 0
		while i < len(valley):
			size = valley[i][1] - valley[i][0]
			if size <= 0:
				size = range_len - size
			if size < min_angle:
				valley.pop(i)
			else:
				i += 1
		angle_choices = []
		for v in valley:
			angle_mean = robot_yaw+msg.angle_min+(v[0]+v[1])/2*msg.angle_max/range_len
			if v[1] <= v[0]:
				angle_mean = angle_mean + np.pi
			angle_choices.append(angle_mean)
		final_angle = None
		for choice in angle_choices:
			if not final_angle or self.angle_cost(choice, robot_yaw) < self.angle_cost(final_angle, robot_yaw):
				final_angle = choice
		self.previous_angle = final_angle
		print("final angle:", final_angle)
		self.target_angle.data = final_angle
		self.target_angle_pub.publish(self.target_angle) """
	
	def scale_to_map(self, x, y):
		return math.floor(x/cell_size), math.floor(y/cell_size)
	
	def angle_cost(self, angle, heading):
		if self.previous_angle:
			return heading_coef*abs(self.angle_diff(heading, angle)) + previous_coef*abs(self.angle_diff(self.previous_angle, angle))
		return heading_coef*abs(self.angle_diff(heading, angle))
	
	def angle_diff(self, angle1, angle2):
		diff = angle1 - angle2
		while not -np.pi <= diff <= np.pi:
			if diff > np.pi:
				diff -= 2*np.pi
			if diff < -np.pi:
				diff += 2*np.pi
		return diff
	
#	def callback_bumper(self, msg):
#		print(msg)
	
	def callback_odometry(self, msg):
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
		self.current = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
	
		
if __name__ == '__main__':
	obstacle_detector = ObstacleDetector()
	while not obstacle_detector.is_finished:
		obstacle_detector.rate.sleep()

