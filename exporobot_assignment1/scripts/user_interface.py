#! /usr/bin/env python

## @package exporobot_assignment1
# \file user_interface.py
# \brief This file contains code for 'user_interface' node.
# \author Shozab Abidi
# \version 1.0
# \date 13/11/2021
#
# \details
#
# Subscribes : <BR>
# 	° /odom
# 	° /control_speed
#
# Publishers : <BR>
# 	° /odom
#
# Service : <BR>
# 	° /go_to_point
#
# This node is user interface of a 2D robot simulation project.
#

import rospy
import time
from exporobot_assignment1.srv import Command, CommandResponse
from std_msgs.msg import String


##  A global variable with datatype 'string' used as a flag to start the simulation.



##
# \brief This is a callback function of the subscriber for the ROS topic '/user_interface_sig'. 
# \param msg is the string data  
# \return [none].
#
# This is function takes the msg data coming from the  '/user_interface_sig' topic and 
# store it in the global variable 'value_.' 
#
#def clbk_expo(msg):
#	global value_
#	value_ = msg.req

##
# \brief This is a 'main' function of user_interface node. 
# 
# \return [none].
#
# This function is a 'main' function of  'user_interface' node. It initializes client for '/user_interface'
# service hosted by 'state_machine' node and subscriber for the 'user_interface_sig' topic. Upon 
# user's request it send the signal to the 'state_machine' node to state the 'random target position
# 'state simulation. 
#
def main():
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
