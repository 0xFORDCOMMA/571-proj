#!/usr/bin/env python
# encoding: utf-8


from 571-proj.srv import *
import rospy
from gen_maze import *
import sys
import argparse
import time

mazeInfo = None

def check_is_edge(edge, valueFlag):
	
	global mazeInfo
	#invalid_edges = mazeInfo[1]
	if valueFlag == "changedValuesLater":
		if edge[2] < mazeInfo[0] or edge[2] > 6*0.5 or edge[3] < mazeInfo[0] or edge[3] > mazeInfo[1]*0.5:
			return False
	elif valueFlag == "changedValuesBefore":
		if edge[0] < mazeInfo[0] or edge[0] >6*0.5 or edge[1] < mazeInfo[0] or edge[1] > mazeInfo[1]*0.5:
			return False
	'''
	if edge in invalid_edges:
		return False
	'''
	else:
		return True

def handle_get_successor(req):
	
	global mazeInfo
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
			g_cost = 2

		elif action == 'TurnCCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index-1)%4]
			g_cost = 2

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
			g_cost = 3
		
		if req.x <= x_cord and req.y <= y_cord:
			isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

		if not isValidEdge:
			state_x.append(-1)
			state_y.append(-1)
			state_direction.append(direction)
			state_cost.append(-1)
		else:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_direction.append(direction)
			state_cost.append(g_cost)

	return GetSuccessorResponse(state_x, state_y, state_direction, state_cost, action_list)
  

def handle_get_initial_state():
	
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])



def proj_server():
    
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    
    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    
    my_maze = Maze()
    
    mazeInfo = my_maze.generate_maze(4)
    project_server()
