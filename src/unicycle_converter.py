#!/usr/bin/env python
"""
Author: Amay Saxena 
"""
from calibration import calibrate, quaternion_matrix
import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import tf
from constants import turtlebot_colors
from calibration import all_active_bots
import math
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d, Axes3D

def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X*(np.pi/180), Y*(np.pi/180), Z*(np.pi/180)

def rot_matrix(phi):
	return np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])

class Robot:
	def __init__(self, turtlebot):
		self.turtlebot = "/"+turtlebot
		self.sub = rospy.Subscriber(self.turtlebot + '/odom', Odometry, self.subscribe)
		self.pub = rospy.Publisher(self.turtlebot+'/mobile_base/commands/velocity', Twist, queue_size=1)

		self.l = 0.1
		self.g = None
		self.xy = np.array([0.0, 0.0])
		self.phi = 0.0
		self.p = self.xy + np.array([self.l, 0.0])
		self.R = np.eye(2)
		self.orientation = None
		self.min_vel = 0.01
		# self.transform = origin_transform

	def subscribe(self, msg):
		trans = np.array([eval('msg.pose.pose.position.' + p) for p in ('x', 'y', 'z')])
		quat = [eval('msg.pose.pose.orientation.' + p) for p in ('x', 'y', 'z', 'w')]
		self.phi = quaternion_to_euler(*quat)[2]
		self.xy = trans[:2]
		self.R = rot_matrix(self.phi)
		self.p = self.xy + self.R[:, 0] * self.l

	def hol_to_uni(self, p_dot):
		bx = self.R[:, 0]
		by = self.R[:, 1]
		v = np.dot(bx, p_dot)
		w = (1.0/self.l) * np.dot(by, p_dot)
		return v, w

	def cmd(self, p_dot):
		print(np.linalg.norm(p_dot))
		if np.linalg.norm(p_dot) > self.min_vel:
			v, w = self.hol_to_uni(p_dot)
		else:
			v, w = 0, 0
		t = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))
		self.pub.publish(t)

	def __str__(self):
		return self.turtlebot

	def __repr__(self):
		return self.turtlebot

class World:
	def __init__(self, sheep, dogs):
		self.sheep = sheep
		self.dogs = dogs
		self.K = 0.1
		self.rate = rospy.Rate(20)

	def potential(self, pos, dogs):
		vel = 0.0
		for dog in dogs:
			xy = dog
			vel += (1.0 / np.linalg.norm(pos - xy) ** 2)
		return self.K * vel

	def velocity(self, pos):
		vel = np.zeros(2)
		for dog in dogs:
			xy = dog.p
			vel += ((pos - xy) / np.linalg.norm(pos - xy) ** 3)
		return self.K * vel

	def execute(self):
		while True:
			for s in self.sheep:
				vel = self.velocity(s.p)
				print vel
				s.cmd(vel)
			self.rate.sleep()

	def visualize(self):
		# dog_pos = np.array([[0,0], [1,0], [0,1]])
		dog_pos = np.array([[-1, -1], [1, -1], [0, 1.73]])#np.array([d.p for d in self.dogs])
		print dog_pos
		off = 0.5
		# x0, x1 = int(min(dog_pos[:, 0]) - off), int(max(dog_pos[:, 0]) + off)
		# y0, y1 = int(min(dog_pos[:, 1])) - off, int(max(dog_pos[:, 1]) + off)

		x0, x1 = min(dog_pos[:, 0]) - off, max(dog_pos[:, 0]) + off
		y0, y1 = min(dog_pos[:, 1]) - off, max(dog_pos[:, 1]) + off
		
		
		res = 0.03

		X = np.arange(x0, x1, step=res)
		Y = np.arange(y0, y1, step=res)
		xx, yy = np.meshgrid(X, Y)
		print xx.shape
		print yy.shape

		Z = np.zeros((xx.shape[0], xx.shape[1]))

		fig = plt.figure()
		ax = Axes3D(fig)#fig.gca(projection='3d')

		for x in range(Z.shape[0]):
			for y in range(Z.shape[1]):
				xi = xx[x, y]
				yi = yy[x, y]
				val = self.potential(np.array([xi, yi]), dog_pos)
				if val > 50.0:
					val = 50.0
				Z[x, y] = val
		print Z
		
		# surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
  #                      linewidth=0, antialiased=False)
  		cset = ax.contour(X, Y, Z, 200, extend3d=True)
  		ax.clabel(cset, fontsize=9, inline=1)
		
		# fig.colorbar(surf, shrink=0.5, aspect=5)
		plt.show()


if __name__ == '__main__':
	rospy.init_node('converter', anonymous=True)

	bots = all_active_bots()
	robots = []
	bots = [turtlebot_colors[int(i)] for i in bots]

	print bots

	sheep = [Robot('pink')]
	dogs = [Robot('black'), Robot('blue'), Robot('green')]

	print "Dogs:", dogs
	print "Sheep:", sheep

	print "MADE ALL BOTS"
	rospy.sleep(3)

	w = World(sheep, dogs)
	print "Waiting"
	# rospy.sleep(10)
	# w.visualize()
	w.execute()

	# i = 0
	# while True:
	# 	pass
	# 	# print '--------------------------'
	# 	# t1 = robots[0].g[:3, 3]
	# 	# t2 = robots[1].g[:3, 3]
	# 	#print "Distance Between Bots: ", np.linalg.norm(t1 - t2) * 39.3701, "inches"
	# 	# print robots[1].g
	# 	# print robots[0].phi / np.pi, "pi radians"
	# 	robots[0].cmd(np.array([0.5, 0.0]))
	# 	rospy.sleep(0.05)	

