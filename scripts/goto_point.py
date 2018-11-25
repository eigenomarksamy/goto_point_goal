#! /usr/bin/env python


import rospy
import numpy as np
from math import pi
from robot_goto_controller import *


def main():

	try:

		turtlebot = robot_goto_controller("goto_point", "turtlebot", 0.0, 0.0, 0.0, 0.0)
		rospy.sleep(1)

		turtlebot.gotoGoal()

	except rospy.ROSInterruptException:

		print "Error Communication: Exception!"


if __name__ == '__main__':
	main()
