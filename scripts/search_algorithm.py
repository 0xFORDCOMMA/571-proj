#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import Api_functions 
import rospy
from std_msgs.msg import String
import argparse
import time

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.", metavar='bfs', action='store', dest='algorithm', default="algorithm", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true', dest='custom_heuristic')


def algorithm(use_custom_heuristic):

    helper = Api_functions.Helper()
    init_state = helper.get_initial_state()
    print(init_state)
    #goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    print "11"
    #visited = []
    state_repr=[]
    q3 = []
    entry_count = 0
    if not helper.is_goal_state(state_repr):
        #visited.append(init_state)
        state_repr.append([init_state.x,init_state.y])
        for key, values in state_dictionary.items():
	    if values[0].x<0 or values[0].y<0:
		continue
            list_1 = []
            temp = []
            list_1.extend(key.split())
            temp.append(list_1)
            temp.append(values[0])
            heapq.heappush(q3, (values[1], entry_count, temp,state_repr))
            entry_count += 1
        while q3:
            cur_node = heapq.heappop(q3)
            if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
                continue
            elif helper.is_goal_state(cur_node[3]):
                print(cur_node[3])
                action_list = cur_node[2][0]
                print(q3)
                break
            #elif cur_node[2][1] in visited:
                #continue
            else:
                temp = helper.get_successor(cur_node[2][1])
		print "22"
		
                #visited.append(cur_node[2][1])
                for key, values in temp.items():
		    if values[0].x<0 or values[0].y<0:
			continue
                    cost = 0
                    state_temp=[]
                    list_1 = []
                    temp = []
                    cost = helper.get_cost(curr_node[2][1],values[0].x,values[0].y)+ cur_node[0]
                    state_temp=cur_node[3]+[[values[0].x,values[0].y]]
                    list_1.extend(cur_node[2][0])
                    list_1.extend(key.split())
                    temp.append(list_1)
                    temp.append(values[0])
                    heapq.heappush(q3, (cost, entry_count, temp,state_temp))
                    entry_count += 1

    return action_list


def exec_action_list(action_list):

    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))
    return action_list


if __name__ == "__main__":
    # DO NOT MODIFY BELOW CODE 
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print("Incorrect Algorithm name.")
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    actions = algorithm(args.custom_heuristic)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)
