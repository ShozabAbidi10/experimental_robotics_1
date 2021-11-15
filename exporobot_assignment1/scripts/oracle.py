#! /usr/bin/env python2

## @package exporobot_assignment1
# \file oracle.py
# \brief This file contains code for oracle node.
# \author Shozab Abidi
# \version 1.0
# \date 13/11/2021
#
# \details
#
# Service : <BR>
# ° /armor_interface_srv
# ° /oracle_service
# 
# This node performs the following tasks: 
# 1. It waits for the request from the motion_controller node to load the hypothesis in the reasonser.
# 2. Start the reasonser and check consistency of the hypothesis.
# 3. Response to the request with a boolen response 'True' if the requested task was achieved or 'False' otherwise.  

import rospy
import time
from exporobot_assignment1.srv import Oracle,OracleResponse
from armor_msgs.srv import ArmorDirective,ArmorDirectiveRequest

armor_client_ = None
armor_req_ = None

count_ = 0
prev_comp_hypo_ = 0


def clbk_oracle_service(msg):
	global count_
	global armor_req_
	global armor_res_
	global prev_comp_hypo_
	
	if(count_ == 0):
		count_ += 1	
		armor_req_ = ArmorDirectiveRequest()
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'LOAD'
		armor_req_.armor_request.primary_command_spec = 'FILE'
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology',
																							 'true', 'PELLET', 'true']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
	if(msg.command == "who" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
		##print("msg.command :", msg.args)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'PERSON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "what" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'WEAPON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
	
	elif(msg.command == "where" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'PLACE']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "REASON" and armor_res_.armor_response.success == True):

		# Starting the reasoner
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'REASON'
		armor_req_.armor_request.primary_command_spec = ''
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = []
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "CONSISTENT" and armor_res_.armor_response.success == True):
		
		# Starting the reasoner
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'QUERY'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = ['COMPLETED']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
					
			new_comp_hypo =  len(armor_res_.armor_response.queried_objects)
			
			print("NEW COMPLETE HYPOTHESIS :", new_comp_hypo)
			print("PREVIOUS COMPLETE HYPOTHESIS :", prev_comp_hypo_)
			
			if(new_comp_hypo > prev_comp_hypo_):
				prev_comp_hypo_ = 	new_comp_hypo		
				return OracleResponse(True)
			else:
				return OracleResponse(False)
	

def main():
	global armor_client_
	rospy.init_node('oracle')
	oracle_server = rospy.Service('/oracle_service', Oracle, clbk_oracle_service)
	armor_client_ = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective)
	rospy.spin()

if __name__ == '__main__':
    main()
