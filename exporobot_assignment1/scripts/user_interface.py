#! /usr/bin/env python


"""
.. module:: user_interface
	:platform: Unix
	:synopsis: Python module for the user Interface

.. moduleauthor:: Shozab Abid hasanshozab10@gmail.com

This node is user interface of the project that communicates with the user and as per the provided commands, instruct the system to behave accordingly. If the user press 1 in the terminal, it request '/user interface' service which is hosted by 'motion_controller.py' node to start the robot simuation.

Service:
	/user_interface

"""

import rospy
import time
from exporobot_assignment1.srv import Command
from std_msgs.msg import String


def main():
	"""

	This is a 'main' function of 'user_interface' node. It initializes
	client for '/user_interface' service hosted by :mod:`motion_controller`. 
	
	Upon user's request the message is passed to the service "user_interface".

	"""
	
	rospy.init_node('user_interface')
	user_client = rospy.ServiceProxy('/user_interface', Command)
	rate = rospy.Rate(10)
	x = int(input("\nPress 1 to start the exploration "))
	while not rospy.is_shutdown():
		if (x == 1):
			print("Exploration started!")
			uc_response_ = user_client("start")		
		if(uc_response_.res == "done"):
			print("Exploration Completed!")
			print("Want to the start exploration again? Press 1 ")
			x = int(input("\nPress 1 to start the robot "))

if __name__ == '__main__':
	main()
