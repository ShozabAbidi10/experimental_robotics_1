#! /usr/bin/env python

"""
.. module:: hint_generator
	:platform: Unix
	:synopsis: Python module for the random hint_generation.

.. moduleauthor:: Shozab Abid hasanshozab10@gmail.com

This node controls the random hint generation task. It waits for the '/request_hint' service's request from the motion controller node. Upon recieving request it randomly generates the hint using a predefined lists of hints and response back to the client with the generated hint. 


Service:
	/request_hint
	
"""

import rospy
import time
from exporobot_assignment1.srv import Hint, HintResponse
from std_msgs.msg import String
from random import randint

 
people = ['Rev. Green','Prof. Plum','Col. Mustard','Mrs. Peacock',
						'Miss. Scarlett', 'Mrs. White']
"""string[]: Initializing global variable 'people' array which is one of the type of hints.

"""

weapons = ['Candlestick','Dagger','Lead Pipe','Revolver',
						'Rope', 'Spanner']
"""string[]: Initializing global variable 'people' array which is one of the type of hints.

"""

places = ['Kitchen','Lounge','Library','Hall','Study', 'Ballroom']
"""string[]: Initializing global variable 'people' array which one of the type of hints.

"""

def clbk_hint_generator(msg):

	"""
	
	This is a 'clbk_hint_generator' function of 'hint_generator' node.
	This function waits for the '/request_hint' service's request from
	the motion controller node. Upon recieving request it randomly 
	generates the hint using a predefined lists of hints and response
	back to the client with the generated hint. 
	
	Args: 
		msg(Hint): the input request message.
		
	Returns: 
		string[]: HintResponse() 
	
	"""	
	
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

	"""
		
	This function is a 'main' function of  'hint_generator' node. It initializes server for '/request_hint' service.
	
	Args: 
		none
				
	Returns: 
		none 	
			
	"""				
	rospy.init_node('hint_generator')
	user_server = rospy.Service('/request_hint', Hint, clbk_hint_generator)
	rospy.spin()

if __name__ == '__main__':
    main()
