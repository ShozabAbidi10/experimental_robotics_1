#! /usr/bin/env python

## @package exporobot_assignment1
# \file hint_generator.py
# \brief This file contains code for hint generator node.
# \author Shozab Abidi
# \version 1.0
# \date 14/11/2021
#
# \details
#
# Subscribes : <BR>
# 	° /
# 	° /
#
# Publishers : <BR>
# 	° /
# 	° /
# 	° /
#   ° /
#
# Service : <BR>
# 	° /
#
# This node control the robot motion. 
# 1. Wait for the hint request from the motion controller node.
# 2. Randomly generate hints from the known list of hints.
# 3. Response to the request with generated hint.  
  
import rospy
import time
from exporobot_assignment1.srv import Hint, HintResponse
from std_msgs.msg import String
from random import randint

people = ['Rev. Green','Prof. Plum','Col. Mustard','Mrs. Peacock',
						'Miss. Scarlett', 'Mrs. White']
weapons = ['Candlestick','Dagger','Lead Pipe','Revolver',
						'Rope', 'Spanner']
places = ['Kitchen','Lounge','Library','Hall','Study', 'Ballroom']
  
def clbk_hint_generator(msg):
	if (msg.req == "need_hint"):
		time.sleep(2)
		x = randint(0,2)
		if x == 0:
			hint = ['who',msg.hypo,people[randint(0,len(people)-1)]]

		elif x == 1:
			hint = ['what',msg.hypo,weapons[randint(0,len(weapons)-1)]]

		elif x == 2:
			hint = ['where',msg.hypo,places[randint(0,len(places)-1)]]
			
		print('Generated Hint :', hint)
		return HintResponse(hint)
		  
	else:
		print("hint request msg is not right.", msg.req)
		return HintResponse([])
		

def main():
    rospy.init_node('hint_generator')
    user_server = rospy.Service('/request_hint', Hint, clbk_hint_generator)
    rospy.spin()

if __name__ == '__main__':
    main()
