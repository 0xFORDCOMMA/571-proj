#!/usr/bin/env python
# encoding: utf-8

#Based on CSE 571 HW 1 - Search
__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import api_functions 
import rospy
from std_msgs.msg import String
import argparse
import time

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {ucs, astar}. Default value is ucs.", metavar='ucs', action='store', dest='algorithm', default="ucs", type=str)
parser.add_argument('-l', help="Length of tour. Possible arguments 1 to 24, inclusive. Defalut is 24", metavar="n", action='store', dest='tour_length', default=24, type=int)


def ucs(tour_length = 24):
    print "Running UCS"
    helper = api_functions.Helper()
    init_state = helper.get_initial_state()
    print(init_state)
    #goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    max_state = 0
    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    state_repr=[]
    q3 = []
    entry_count = 0
    local_visited=[]
    all_states = helper.get_all_states()
    all_states = set([tuple(l) for l in helper.get_all_states()])
    
    state_repr.append([init_state.x,init_state.y])
    for key, values in state_dictionary.items():
        if values.x<0 or values.y<0:
            continue
        local_visited=[]
        list_1 = []
        temp = []
        list_1.extend(key.split())
        temp.append(list_1)
        temp.append(values)
        local_visited.append(init_state)
        cost=helper.get_cost(init_state,values.x,values.y,key)
        heapq.heappush(q3, (cost, entry_count, temp,state_repr,local_visited ))
        entry_count=entry_count+1
    while q3:
        cur_node = heapq.heappop(q3)
        # print "cur_list" + str(cur_node[3])
        #for x in q3:
            #print "q3 right now" + str(x)
        if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
            #print "reject: " + str(cur_node)
            continue
        elif len(set([tuple(l) for l in cur_node[3]])) >= tour_length:
        #elif len(all_states.difference(set([tuple(v) for v in cur_node[3]]))) < 14:
            print("goal_reached")
            print init_state
            print cur_node[0]
            action_list = cur_node[2][0]
            break
        else:
            for visited in cur_node[4]:
                if cur_node[2][1] == visited:
                    continue
            temp = helper.get_successor(cur_node[2][1])
            
            for key, values in temp.items():
                if values.x<0 or values.y<0:
                    continue
                cost = 0
                state_temp=[]
                local_visited=[]
                list_1 = []
                temp = []
                
                state_temp=cur_node[3]+[[values.x,values.y]]
                local_visited=cur_node[4]+[cur_node[2][1]]
                list_1.extend(cur_node[2][0])
                list_1.extend(key.split())
                cost=helper.get_cost(init_state,values.x,values.y,key) + cur_node[0]
                temp.append(list_1)
                temp.append(values)
                next = (cost, entry_count, temp,state_temp,local_visited)
                if len(set([tuple(l) for l in state_temp])) > max_state:
                    max_state = len(set([tuple(l) for l in state_temp]))
                    print "Max Node" + str(max_state)
                #print next
                heapq.heappush(q3, next)
                entry_count += 1
        #print "q3 size:" + str(len(q3))
    return action_list


def astar(tour_length):
    print "Running A*"
    helper = api_functions.Helper()
    init_state = helper.get_initial_state()
    print(init_state)
    #goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    max_state = 0
    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    state_repr=[]
    q3 = []
    entry_count = 0
    local_visited=[]
    all_states = helper.get_all_states()
    all_states = set([tuple(l) for l in helper.get_all_states()])
    
    state_repr.append([init_state.x,init_state.y])
    for key, values in state_dictionary.items():
        if values.x<0 or values.y<0:
            continue
        local_visited=[]
        list_1 = []
        temp = []
        list_1.extend(key.split())
        temp.append(list_1)
        temp.append(values)
        local_visited.append(init_state)
        heuristic_cost=helper.get_heuristic(state_repr,[values.x,values.y])
        final_cost=heuristic_cost+helper.get_cost(init_state,values.x,values.y,key)
        cost=helper.get_cost(init_state,values.x,values.y,key)
        heapq.heappush(q3, (final_cost , entry_count, temp,state_repr,local_visited, cost))
        entry_count=entry_count+1
    while q3:
        cur_node = heapq.heappop(q3)
        # print "cur_list" + str(cur_node[3])
        #for x in q3:
            #print "q3 right now" + str(x)
        if cur_node[2][1].x < 0 or cur_node[2][1].y < 0:
            #print "reject: " + str(cur_node)
            continue
        elif len(set([tuple(l) for l in cur_node[3]])) >= tour_length:
        #elif len(all_states.difference(set([tuple(v) for v in cur_node[3]]))) < 14:
            print("goal_reached")
            print init_state
            print cur_node[0]
            action_list = cur_node[2][0]
            break
        else:
            for visited in cur_node[4]:
                if cur_node[2][1] == visited:
                    continue
            temp = helper.get_successor(cur_node[2][1])
            
            for key, values in temp.items():
                if values.x<0 or values.y<0:
                    continue
                cost = 0
                state_temp=[]
                local_visited=[]
                list_1 = []
                temp = []
                
                state_temp=cur_node[3]+[[values.x,values.y]]
                local_visited=cur_node[4]+[cur_node[2][1]]
                list_1.extend(cur_node[2][0])
                list_1.extend(key.split())
                heuristic_cost = helper.get_heuristic(state_temp, [values.x, values.y])
                cost=helper.get_cost(init_state,values.x,values.y,key) + cur_node[5]
                final_cost = heuristic_cost + cost
                temp.append(list_1)
                temp.append(values)
                next = (final_cost, entry_count, temp,state_temp,local_visited,cost)
                if len(set([tuple(l) for l in state_temp])) > max_state:
                    max_state = len(set([tuple(l) for l in state_temp]))
                    print "Max Node" + str(max_state)
                #print next
                heapq.heappush(q3, next)
                entry_count += 1
        #print "q3 size:" + str(len(q3))
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

    start_time = time.time()
    actions = algorithm(args.tour_length)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)