import os
import math
def nav_line(x, y = 0, speed = 0.1):
	"""A primitive method to move the base straight.

	:param x: Define how many meter you want to move the base in front of or behind
	:param y: Define how many meter you want to move the base on the right or on the left
	:param speed: Define the speed will move the base
	"""

        goal = os.popen("/u/ncourbet/openrobots/src/ros-nodes/navGoto/bin/navLine " + str(x) + " " +  str(y) + " 0  base_link " + str(speed))

        print goal

        for line in goal.readlines():
                print (line)


def rotation(theta):
	"""A primitive method to go round in circles

	:param theta: The number of degree you want to turn the robot

	"""

	theta_rd = math.radians(float(theta))

        goal = os.popen("/u/ncourbet/openrobots/src/ros-nodes/navGoto/bin/simpleGoal 0  0 " + str(theta_rd) +  " base_link")

        print goal

        for line in goal.readlines():
                print (line)

