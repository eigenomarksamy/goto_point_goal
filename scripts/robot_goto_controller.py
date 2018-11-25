#! /usr/bin/env python


import math
import time
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties


class robot_goto_controller:


	def __init__(self, node_name, name, X, Y, PSI, yawr):

		self._name = name
		self._node_name = node_name
		self._x = X
		self._y = Y
		self._psi = PSI
		self._yawr = yawr

		rospy.init_node(self._node_name, anonymous = True)

		self._rate = rospy.Rate(10)

		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size = 10, latch = True)
		rospy.sleep(1)
		self._jointPropertiesGetter = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		rospy.wait_for_service('/gazebo/get_joint_properties')
		self._jointEffortApplier = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
		rospy.wait_for_service('/gazebo/apply_joint_effort')
		self._modelStateGetter = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		rospy.wait_for_service('/gazebo/get_model_state')

		self._msg = ModelState()

		self._msg.model_name = self._name

		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._y
		self._msg.pose.position.z = 0.0
		self._msg.pose.orientation.x = 0.0
		self._msg.pose.orientation.y = 0.0
		self._msg.pose.orientation.z = self._psi
		self._msg.pose.orientation.w = 0.0
		self._msg.twist.linear.x = 0.0
		self._msg.twist.linear.y = 0.0
		self._msg.twist.linear.z = 0.0
		self._msg.twist.angular.x = 0.0
		self._msg.twist.angular.y = 0.0
		self._msg.twist.angular.z = self._yawr

		self._modelStateSetter.publish(self._msg)


	def getJointProperties(self, joint_name):

		joints = self._jointPropertiesGetter(joint_name)
		return joints


	def applyJointEffor(self, joint_name, effort, duration):
		
		start_time = rospy.get_rostime()
		self._jointEffortApplier(joint_name, effort, start_time, duration)


	def setPosition(self, X, Y, PSI):

		self._x = X
		self._y = Y
		self._psi = PSI

		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._y
		self._msg.pose.orientation.z = self._psi

		self._modelStateSetter.publish(self._msg)


	def setVelocity(self, x_dot, y_dot, yawr):

		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._y
		self._msg.pose.orientation.z = self._psi

		self._msg.twist.linear.x = x_dot
		self._msg.twist.linear.y = y_dot
		self._msg.twist.angular.z = yawr

		self._modelStateSetter.publish(self._msg)


	def calcDistance(self, goal_x, goal_y):

		distance = (((goal_x - self._x)**2) + ((goal_y - self._y)**2))**0.5
		print distance
		return distance


	def calcLinearVelocity(self, goal_x, goal_y, constant = 3.0):

		linear_velocity = constant * self.calcDistance(goal_x, goal_y)
		return linear_velocity


	def calcSteeringAngle(self, goal_x, goal_y):

		steering_angle = math.atan2(goal_y - self._y, goal_x - self._x)
		return steering_angle


	def calcAngularVelocity(self, goal_x, goal_y, constant = 6.0):

		angular_velocity = constant * (self.calcSteeringAngle(goal_x, goal_y) - self._psi)
		return angular_velocity


	def updateModelState(self):

		state = self._modelStateGetter(self._name, "world")

		self._x = state.pose.position.x
		self._y = state.pose.position.y
		self._psi = state.pose.orientation.z


	def gotoGoal(self):

		goal_x = input("Desired X goal: ")
		goal_y = input("Desired Y goal: ")
		distance_tolerance = 0.7

		while self.calcDistance(goal_x, goal_y) >= distance_tolerance:

			x_velocity = self.calcLinearVelocity(goal_x, goal_y)
			y_velocity = 0.0
			z_velocity = 0.0

			yawr_velocity = self.calcAngularVelocity(goal_x, goal_y)

			self.setVelocity(x_velocity, y_velocity, yawr_velocity)

			self._rate.sleep()
			self.updateModelState()

		print "Reached GOAL!"

		x_velocity = 0.0
		y_velocity = 0.0
		yawr_velocity = 0.0

		self.setVelocity(x_velocity, y_velocity, yawr_velocity)
