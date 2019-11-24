#!/usr/bin/env python



from group_13.srv import *
import rospy

import sys
import argparse
import time



def check_is_edge(edge, valueFlag):
	
	
	
	if valueFlag == "changedValuesLater":
		if edge[2] < 0 or edge[2] >6  or edge[3] < 0 or edge[3] > 4:
			return False
	elif valueFlag == "changedValuesBefore":
		if edge[0] < 0 or edge[0] >6 or edge[1] < 0 or edge[1] > 4:
			return False
	
def handle_get_successor(req):
	
	
	action_list = ["TurnCW", "TurnCCW", "MoveB", "MoveF"]
	direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
	state_x = []
	state_y = []
	state_direction = []
	state_cost = []
	
	for action in action_list:
		#Checking requested action and making changes in states
		x_cord, y_cord, direction = req.x, req.y, req.direction
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
			g_cost = 1

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
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

		if isValidEdge:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_direction.append(direction)
			state_cost.append(g_cost)

	return GetSuccessorResponse(state_x, state_y, state_direction, state_cost, action_list)
  



def project_server():
    
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    
    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    
    
    project_server()
