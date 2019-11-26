#!/usr/bin/env python



from group_13.srv import *
import gen_init_state
import rospy
import random
import sys
import argparse
import time
import json
import os
import math


def check_is_edge(edge, valueFlag):
	
	
	
	if valueFlag == "changedValuesLater":
		if edge[2] < 0 or edge[2] >5  or edge[3] < 0 or edge[3] > 3:
			return False
		else: 
			return True
	elif valueFlag == "changedValuesBefore":
		if edge[0] < 0 or edge[0] >5 or edge[1] < 0 or edge[1] > 3:
			return False
		else:
			return True
	


def handle_get_initial_state(req):
	
	global init_state

	initial_state = init_state
	return GetInitialStateResponse(initial_state[0],initial_state[1],initial_state[2])
	
def handle_get_successor(req):
	
	#print "hello"
	action_list = ["TurnCW", "TurnCCW", "MoveB", "MoveF"]
	direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
	state_x = []
	state_y = []
	
	state_direction = []
	
	#print "hello"
	for action in action_list:
		#Checking requested action and making changes in states
		x_cord, y_cord, direction = req.x, req.y, req.direction
		#print x_cord
		#print y_cord
		if action == 'TurnCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index+1)%4]
			

		elif action == 'TurnCCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index-1)%4]
			

		elif action == 'MoveF':
			if direction == "NORTH":
				y_cord += 1
			elif direction == "EAST":
				x_cord += 1
			elif direction == "SOUTH":
				y_cord -= 1
			elif direction == "WEST":
				x_cord -= 1
			
		elif action == 'MoveB':
			if direction == "NORTH":
				y_cord -=1
			elif direction == "EAST":
				x_cord -= 1
			elif direction == "SOUTH":
				y_cord += 1
			elif direction == "WEST":
				x_cord += 1
			
			

		if req.x <= x_cord and req.y <= y_cord:
			isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
			#print isValidEdge
			#print "valid later"
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")
			#print isValidEdge
			#print "valid before"

		if not isValidEdge:
			state_x.append(-1)
			state_y.append(-1)
			
			state_direction.append(direction)
			#print "hello"
			
		else:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_direction.append(direction)
			
		#print x_cord
		#print y_cord
	return GetSuccessorResponse(state_x, state_y, state_direction,action_list)

def project_server():
    
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)

      

    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    
    
    init_state=gen_init_state.get_rand_initial_state()
    print init_state
    project_server()
