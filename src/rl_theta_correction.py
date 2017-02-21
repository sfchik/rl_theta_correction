#!/usr/bin/env python
## A theta correction script
## to solve bug where robot orientation is not updated 
## during rotation without any linear motion

import rospy
import math
import time
import tf
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

PI = 3.1416
HZ = 100.0
HZ_RATIO = 10

class ThetaCorrection:
	def __init__(self):
		self.r = rospy.Rate(HZ)
		#last thing to do before shutting down
#		rospy.on_shutdown(self.cleanup)
		
		#initializing message
		self.msg_robot_odometry = Odometry()
		self.msg_robot_odometry_corrected = Odometry()
		self.msg_robot_twist = Twist()
		self.msg_robot_twist_current = Twist()

		#initializing message with specific values
		#TO DO - change it to general
		self.theta_robot = 90.0 #only at certain scenario

		#initialize variables
		self.yaw_prev_ = 0.0
		self.omega_prev_ = 0.0

		#Subscriber setup
		rospy.Subscriber('/pedsim/robot_position', Odometry, self.callbackRobotOdometry)
		rospy.Subscriber('pedbot/control/cmd_vel', Twist, self.callbackRobotTwist)	

		#Publisher setup
		self.pub_robot_position_ = rospy.Publisher('/robot_position/corrected', Odometry, queue_size=10)
		# self.pub_robot_position_costmap_ = rospy.Publisher('/robot_position/for_costmap', Odometry, queue_size=10)
		# self.pub_robot_twist_ = rospy.Publisher('pedbot/control/cmd_vel', Odometry, queue_size=10)

		#Dummy flag setup
		self.FLAG_START_ODOM = False
		self.FLAG_START_TWIST = False
		self.FLAG_UPDATE_DONE = False

		#Dummy counter
		self.count_print_ = 0

	def loop(self):
#		 this section is the correction for in place rotation
		 while not rospy.is_shutdown():
		 	if self.FLAG_START_ODOM is True and self.FLAG_START_TWIST is True:
		 		quaternion = [self.msg_robot_odometry.pose.pose.orientation.x, self.msg_robot_odometry.pose.pose.orientation.y, \
		 				self.msg_robot_odometry.pose.pose.orientation.z, self.msg_robot_odometry.pose.pose.orientation.w] 
		 		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion) #convert quarternion data into eular					
		 		if self.msg_robot_twist.linear.x < 0.01 and self.msg_robot_twist.linear.x > -0.01 \
		 			and (self.msg_robot_twist.angular.z > 0.01 or self.msg_robot_twist.angular.z < -0.01):	
		 			#this is where lies the bug where linear motion is 0.0, the robot orientation is not updated
		 			#use angular velocity to update robot orientation
		 			self.theta_robot += 180.0 *((self.msg_robot_twist.angular.z)/3.1416) * (1.0/HZ)
		 			#reset value after one round
		 			if self.theta_robot > 360.0001:  
		 				self.theta_robot = 0.0
   				#reset value after one round 
		 			elif self.theta_robot < -0.0001:
		 				self.theta_robot = 360.0			
		 		elif self.msg_robot_twist.linear.x > 0.01 or self.msg_robot_twist.linear.x < -0.01:
		 			self.theta_robot = 180.0*(yaw/PI)
		 			if self.theta_robot < -0.0001:
		 				self.theta_robot = 360.0 + self.theta_robot	#to make it goes from 0 to 360 instead of 180 to -180

		 		# self.count_print_ = self.count_print_ + 1
			 	# if self.count_print_ > HZ_RATIO:	
			 	# 	if self.yaw_prev_ < -0.001 and yaw < -0.001:
			 	# 		omega = (yaw - self.yaw_prev_)*float(HZ/HZ_RATIO)
			 	# 	elif self.yaw_prev_  > 0.001 and yaw > 0.001:
			 	# 		omega = (yaw - self.yaw_prev_)*float(HZ/HZ_RATIO)
			 	# 	else:		
					# 	omega = self.omega_prev_
					# self.omega_prev_ = omega	
			 	# 	self.yaw_prev_ = yaw	
			 	# 	# print self.msg_robot_odometry.twist.twist.linear.x, self.msg_robot_odometry.twist.twist.linear.y, omega, yaw
			 	# 	self.count_print_ = 0	
			 					
			 	#TO DO - use other method instead of borrowing the position.z to transfer msg			
			 	self.msg_robot_odometry.pose.pose.position.z = self.theta_robot
			 	# self.pub_robot_position_costmap_.publish(self.msg_robot_odometry)
			 	self.msg_robot_odometry.twist.twist.linear.x = math.hypot(self.msg_robot_odometry.twist.twist.linear.x, self.msg_robot_odometry.twist.twist.linear.y)
			 	self.msg_robot_odometry.twist.twist.linear.y = 0.0
			 	self.msg_robot_odometry.twist.twist.angular.z = self.msg_robot_twist.angular.z
			 	# self.msg_robot_odometry.twist.twist.angular.z = self.omega_prev_
			 	self.pub_robot_position_.publish(self.msg_robot_odometry)

		 	self.r.sleep()

		#testing for directly combine position and speed cmd into odom to be publish as robot speed and position
#		while not rospy.is_shutdown():
#			if self.msg_robot_twist.linear.x > self.msg_robot_twist_current.linear.x:
#				self.msg_robot_twist_current.linear.x = min(self.msg_robot_twist.linear.x, \
#																	self.msg_robot_twist_current.linear.x + 1.0*0.01)
#			elif self.msg_robot_twist.linear.x < self.msg_robot_twist_current.linear.x:
#				self.msg_robot_twist_current.linear.x = max(self.msg_robot_twist.linear.x, \
#																	self.msg_robot_twist_current.linear.x - 1.0*0.01)		
#			else:
#				self.msg_robot_twist_current.linear.x = self.msg_robot_twist.linear.x

#			if self.msg_robot_twist.angular.z > self.msg_robot_twist_current.angular.z:
#				self.msg_robot_twist_current.angular.z = min(self.msg_robot_twist.angular.z, \
#																	self.msg_robot_twist_current.angular.z + 2.0*0.01)
#			elif v_cmd < v_current:
#				self.msg_robot_twist_current.angular.z = max(self.msg_robot_twist.angular.z, \
#																	self.msg_robot_twist_current.angular.z - 2.0*0.01);	
#			else:
#				self.msg_robot_twist_current.angular.z = self.msg_robot_twist.angular.z

#			self.pub_robot_twist_.publish(self.msg_robot_twist_current)

#			self.msg_robot_odometry.twist.twist.linear.x = self.msg_robot_twist_current.linear.x
#			self.msg_robot_odometry.twist.twist.angular.z = self.msg_robot_twist_current.angular.z
#			self.pub_robot_position_.publish(self.msg_robot_odometry)
#			self.r.sleep()


	def callbackRobotOdometry(self, msg):
		self.FLAG_START_ODOM = True
		self.msg_robot_odometry = msg

	def callbackRobotTwist(self, msg):
		self.FLAG_START_TWIST = True
		self.msg_robot_twist = msg

if __name__=="__main__":
	rospy.init_node('rl_theta_correction')
	tc = ThetaCorrection()
	try:
		tc.loop()
	except:
		pass
