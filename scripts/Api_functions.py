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

class State:
    
    
	def __init__(self,x,y,orientation):
        
		self.x  = x
		self.y = y
		self.orientation = orientation

	def __eq__(self, other):
		if self.x == other.x and self.y == other.y :
		    return True
		else:
		    return False
	def __repr__(self):
        	return "({}, {}, {})".format(str(self.x), str(self.y), str(self.orientation))

class Helper:
	def get_initial_state():
		
		return State(0,0,'East')
        	
	def get_successor (self, curr_state):
        
		rospy.wait_for_service('get_successor')

		try:
			get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
		    	response = get_successor(curr_state.x, curr_state.y, curr_state.orientation)
		    	states = collections.OrderedDict()

			for i in range(len(response)):
		        	states[response.action[i]] = (State(response.x[i], response.y[i], response.direction[i]), response.g_cost[i])
			return states
		
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

	def get_actions(self):
        
        	return ["TurnCW", "TurnCCW", "MoveF", "MoveB"]
	
	def get_all_states(self):
		
		all_states=[]
		for i in range(0,7):
			for j in range(0,5):
				all_states.append((i,j))
		return all_states

	def is_goal_state(self,visited)
		
		all_state=self.get_all_states()
		dup=all_state
		for st in visited:
			if(st in dup):
				dup.remove(st)
		if not dup:
			return True
		else:
			return False
								

	def usage(self):
        	return "%s [x y]" % sys.argv[0]



