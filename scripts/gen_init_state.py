from group_13.srv import *
import rospy
import random
import sys



def get_rand_initial_state():
		'''
		#return State(0,0,'East')
		rospy.wait_for_service('get_rand_initial_state')
		try:
		    get_rand_initial_state = rospy.ServiceProxy('get_rand_initial_state', GetRandInitialState)
		    response = get_rand_initial_state()
		    return State(response.x, response.y, response.direction)

		except rospy.ServiceException, e:
		     print "Service call failed: %s" % e
		'''
		direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
		x=random.randrange(0,6,1)
		y=random.randrange(0,4,1)

		direction=random.choice(direction_list)
		initial_state = [x,y,direction]
		return initial_state
