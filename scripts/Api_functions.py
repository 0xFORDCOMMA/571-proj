#!/usr/bin/env python
# encoding: utf-8

import sys
import rospy

from group_13.srv import *
import collections
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from pid import PID
import copy
import os
import json

class State:
    
    
	def __init__(self,x,y,direction):
        
		self.x  = x
		self.y = y
		self.direction = direction

	def __eq__(self, other):
		if self.x == other.x and self.y == other.y :
		    return True
		else:
		    return False
	def __repr__(self):
        	return "({}, {}, {})".format(str(self.x), str(self.y), str(self.direction))

class Helper:
	
	def get_cost(self,curr_state,v1,v2,key):
		root_path=os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
		with open(root_path + "/reef.json") as reef_file:
			try:
				reef_obj=json.load(reef_file)
			except (ValueError, KeyError, TypeError):
               			print "JSON error"
		print "key: " + str(key)
		
		if key=="TurnCW" or key=="TurnCCW":
			g_cost=6
			
		elif key=="MoveB":
			x1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['x'])
			y1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['y'])
			z1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['z'])

			x2=float(reef_obj[int(v1)][int(v2)]['x'])
			y2=float(reef_obj[int(v1)][int(v2)]['y'])
			z2=float(reef_obj[int(v1)][int(v2)]['z'])

			g_cost=2*(math.sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2)))
		elif key=="MoveF":
		
			x1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['x'])
			y1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['y'])
			z1=float(reef_obj[int(curr_state.x)][int(curr_state.y)]['z'])

			x2=float(reef_obj[int(v1)][int(v2)]['x'])
			y2=float(reef_obj[int(v1)][int(v2)]['y'])
			z2=float(reef_obj[int(v1)][int(v2)]['z'])
		
					

			g_cost=math.sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2))
		#print key
		#print g_cost
		return g_cost
	

	def get_initial_state(self):
		
		#return State(0,0,'East')
		rospy.wait_for_service('get_initial_state')
		try:
		    get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
		    response = get_initial_state()
		    return State(response.x, response.y, response.direction)

		except rospy.ServiceException, e:
		     print "Service call failed: %s" % e
        	
	def get_successor (self, curr_state):
        
		rospy.wait_for_service('get_successor')

		try:
			get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
		    	response = get_successor(curr_state.x,curr_state.y,curr_state.direction)
			#print response
		    	states = collections.OrderedDict()
			d={}
		
			for i in range(4):
				d={response.action[i] : State(response.x[i], response.y[i], response.direction[i])}
		        	states.update(d)
		

			return states
		
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

	def get_actions(self):
        
        	return ["TurnCW", "TurnCCW", "MoveF", "MoveB"]
	
	def get_all_states(self):
		
		all_states=[]
		for i in range(0,6):
			for j in range(0,4):
				all_states.append((i,j))
		return all_states

	def is_goal_state(self,visited):
		Total_state=set([tuple(l) for l in self.get_all_states()])
		visited_set = set([tuple(v) for v in visited])

		return len(Total_state.difference(visited_set)) == 0

		#dup=Total_state
		#for st in visited:
	#		if st in dup:
	#			dup.remove(st)
	#	if not dup:
	#		return True
	#	else:
	#		return False
		
	def usage(self):
        	return "%s [x y]" % sys.argv[0]


