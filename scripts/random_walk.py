#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
from api_functions import * 
import rospy
from std_msgs.msg import String
import numpy as np
import random

class RandomWalk:

    def __init__(self):
        self.helper = Helper()
        rospy.init_node("random_walk")
        self.publisher = rospy.Publisher("/actions", String, queue_size = 10)
        self.subscriber = rospy.Subscriber("/status", String, self.callback)
        self.init_state = self.helper.get_initial_state()
        self.current_state = self.init_state
        self.last_action = None
        self.visited_list = set()
        rospy.Rate(1).sleep()
        print "Running"
        self.next_action()
        rospy.spin()

# temp = helper.get_successor(cur_node[2][1])
#                 visited.append(cur_node[2][1])
#                 for key, values in temp.items():
#                     cost = 0
#                     state_temp=[]
#                     list_1 = []
#                     temp = []
#                     cost = values[1] + cur_node[0]
#                     state_temp=cur_node[3]+[values[0]]
#                     list_1.extend(cur_node[2][0])
#                     list_1.extend(key.split())
#                     temp.append(list_1)
#                     temp.append(values[0])
#                     heapq.heappush(q3, (cost, entry_count, temp,state_temp))



    def random_walk(self):
        '''
        Randomly choses an action to perform among possible actions
        '''
        helper = Helper()
        current_state = helper.get_initial_state()
        while True:
            possible_next_states = helper.get_successor(current_state)

            candidates = []
            for key, values in possible_next_states.items():
                if values.x<0 or values.y<0:
                    continue
                candidates.append((key,values))
            selected = random.choice(candidates)
            self.visited_list.add(tuple(selected[0]))
            return selected[0], selected[1]



    def next_action(self):
        '''
        Updates current state from chosen action and publishes the action to the /actions topic
        '''
        if self.helper.is_goal_state(self.visited_list):
            print "Goal Reached"
            self.publisher.publish(String(data = 'land'))

        else:
            next_state, action = self.random_walk()
            print "Executing ", action
            self.current_state = next_state
            action_str = String(data = action)
            self.publisher.publish(action_str)

    def callback(self,data):
        '''
        callback for handling status messages of turtlebot
        executes the next action when the turtlebot is ready
        '''
        if data.data == "next":
            self.next_action()


if __name__ == "__main__":
    random_walker = RandomWalk()