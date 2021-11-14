#! /usr/bin/env python

## @package exporobot_assignment1
# \file motion_controller.py
# \brief This file contains code for controlling motion of the robot.
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
# 	° /control_speed
# 	° /cancel_goals
# ° /success_goals
#
# Service : <BR>
# ° /go_to_point
#
# This node control the robot motion. 
# 1. Wait for the oracle to request to start the exploration. 
# 2. When the request is recieved start the while loop in which it visit all the three rooms. 
# 3. All the three rooms coordinates are predefined: R1(2,0) R2(0,2) R3(-2,0). Robot start the exploration from start position P(1,1)
# 4. After reaching every room coordinates, it request the oracle or someone to give him the hint. 
# 5. After visiting and reciving all the hints it start the reasoner. 
# 6. Then it check if the hypothesis based on previous loaded hints is consistent or not. 
# 7. If the hints are consistent then it go to the origin position O(0,0) and print the statement. 
# 8. Ask the user_interface node to ask user to again do the exploration. 
# 9. If its not consistent then it will continue the exploration again. 
  
  
import rospy
import time
from exporobot_assignment1.srv import Command, CommandResponse
from std_msgs.msg import String
from geometry_msgs.msg import Point

 
  
def clbk_user_interface(msg):
	if (msg.req == "start"):
		 time.sleep(5)
		 print("robot completed exploration!!")
		 return exploration()
	else:
		print("req msg is not right.", msg.req)
		return CommandResponse("not done")
		
def exploration():
	robot_curr_pos = Point()
	robot_curr_pos.x = 1
	robot_curr_pos.y = 1
	
	room1 = Point()
	room1.x = 2
	room1.y = 0

	room2 = Point()
	room2.x = 0
	room2.y = 2

	room3 = Point()
	room3.x = -2
	room3.y = 0
	
	origin = Point()
	origin.x = 0
	origin.y = 0	
	
	while 1:

		if(visit(robot_curr_pos, room1)):
			robot_curr_pos.x,robot_curr_pos.y = room1.x,room1.y
			rospy.loginfo('Robot is in Room1 (%i,%i). Now collecting hint.',robot_curr_pos.x,robot_curr_pos.y)
			time.sleep(2)
			# Now call hint service
			rospy.loginfo('Collected hint from room1.')
			# Now call hint load service
			rospy.loginfo('Loading the hint in reasoner.')
		else:
			rospy.loginfo('Robot was not able to reach room1.')
			return("not done")
			break
	
		if(robot_curr_pos.x,robot_curr_pos.y == room1.x,room1.y):
			if(visit(robot_curr_pos, room2)):
				robot_curr_pos.x,robot_curr_pos.y = room2.x,room2.y
				rospy.loginfo('Robot is in Room2 (%i,%i). Now collecting hint.',robot_curr_pos.x,robot_curr_pos.y)
				time.sleep(2)
				# Now call hint service
				rospy.loginfo('Collected hint from room2.')
				# Now call hint load service
				rospy.loginfo('Loading the hint in reasoner.')
			else:
				rospy.loginfo('Robot was not able to reach room2.')
				return("not done")
				break

		if(robot_curr_pos.x,robot_curr_pos.y == room2.x,room2.y):
			if(visit(robot_curr_pos, room3)):
				robot_curr_pos.x,robot_curr_pos.y = room3.x,room3.y
				rospy.loginfo('Robot is in Room3 (%i,%i). Now collecting hint.',robot_curr_pos.x,robot_curr_pos.y)
				time.sleep(2)
				# Now call hint service
				rospy.loginfo('Collected hint from room3.')
				# Now call hint load service
				rospy.loginfo('Loading the hint in reasoner.')
			else:
				rospy.loginfo('Robot was not able to reach room3.')
				return("not done")
				break			

		if(robot_curr_pos == room3):
			# Starting reasoner
			rospy.loginfo('Starting the reasoner.')
			
			# Checking if inconsistency > prev_inconsistency:
			rospy.loginfo('Checking the consistency..')
			time.sleep(3)
			rospy.loginfo('Hypothesis found conistence. Now going to origin.')
			if(visit(robot_curr_pos, origin)):
				rospy.loginfo('Robot reached origin (%i,%i).',robot_curr_pos.x,robot_curr_pos.y)
				rospy.loginfo('Printing statement..')
				return("done")
			else:
				rospy.loginfo('Robot was not able to reach origin.')
				return("not done")
				break
	
def visit(current,goal):

	curr_pos = Point()
	curr_pos.x, curr_pos.y =  current.x,current.y
	dist = ((((goal.x - curr_pos.x)**2)+((goal.y - curr_pos.y)**2))**0.5)
	dist = round(dist,2)
	vel_factor = 0.1
	while not dist <= 0.05:
		
		if (goal.x-curr_pos.x)>0:	vel_x = vel_factor*1 
		else:  vel_x = vel_factor*-1
		
		if (goal.y-curr_pos.y)>0:	vel_y = vel_factor*1 
		else:  vel_y = vel_factor*-1
		
		curr_pos.x += round(vel_x,2)
		curr_pos.y += round(vel_y,2)
		dist = ((((goal.x - curr_pos.x)**2)+((goal.y - curr_pos.y)**2))**0.5)
		dist = round(dist,2)	
		rospy.loginfo('Robot moving (%f,%f), dist(%f)',curr_pos.x,curr_pos.y,dist)	
		time.sleep(1)
		
	if(dist <= 0.05):
		return True
	else:
		return False
	
	
def main():
    global pub_
    rospy.init_node('motion_controller')
    user_server = rospy.Service('/user_interface', Command, clbk_user_interface)
    rospy.spin()

if __name__ == '__main__':
    main()
