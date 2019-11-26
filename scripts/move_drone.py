#!/usr/bin/env python
# encoding: utf-8

# based on move_tbot3.py
__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from std_msgs.msg import String
import math
import copy, json, time, os
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import api_functions as api
def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)


def get_offsets(grid, x, y):
    return float(grid[int(x)][int(y)]['x'])+6, float(grid[int(x)][int(y)]['y'])+10, float(grid[int(x)][int(y)]['z'])+5.5


class moveDrone:
    def init_drone(self):
        connection_string = '127.0.0.1:14540'
        MAV_MODE_AUTO = 4

        print "Connecting"
        self.vehicle = connect(connection_string, wait_ready=True)

        def PX4setMode(mavMode):
            self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system, self.vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

        #Create a message listener for self.home position fix
        @self.vehicle.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            global home_position_set
            self.home_position_set = True

        print "Home Location Set %s" % self.vehicle.home_location

        # Display basic self.vehicle state
        #print " Type: %s" % self.vehicle.vehicle_type
        print " Armed: %s" % self.vehicle.armed
        print " System status: %s" % self.vehicle.system_status.state
        print " GPS: %s" % self.vehicle.gps_0
        print " Alt: %s" % self.vehicle.location.global_relative_frame.alt

        # Change to AUTO mode
        PX4setMode(MAV_MODE_AUTO)
        time.sleep(1)

        # Load commands
        cmds = self.vehicle.commands
        cmds.clear()

        self.home = self.vehicle.location.global_relative_frame

        #takeoff to starting point
        wp = get_location_offset_meters(self.home, 0, 0, 10);
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = get_location_offset_meters(self.home, *get_offsets(self.grid, self.loc.x, self.loc.y))
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        # Upload mission
        cmds.upload()
        time.sleep(2)

        # Arm self.vehicle
        self.vehicle.armed = True

        # monitor mission execution
        nextwaypoint = self.vehicle.commands.next
        while nextwaypoint < len(self.vehicle.commands):
            if self.vehicle.commands.next > nextwaypoint:
                display_seq = self.vehicle.commands.next+1
                print "Moving to waypoint %s" % display_seq
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)


    def __init__(self):
        rospy.init_node('move_drone',anonymous = True)
        self.actions = String()
        self.helper = api.Helper()
        self.loc = self.helper.get_initial_state()
        self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
        self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
        self.free = String(data = "next")
        self.rate = rospy.Rate(30)
        self.root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        with open(self.root_path + '/reef.json', 'r') as f:
            self.grid = json.load(f)
        self.init_drone()
        print("Ready!")
        rospy.spin()

    def callback_actions(self,data):
        self.actions = data.data.split("_")
        self.rate.sleep()
        for i in range(len(self.actions)):
            self.execute_next()
            time.sleep(5)

    def execute_next(self):
        action = self.actions.pop(0)
        if action == "MoveF" or action == "MoveB":
            current_loc = self.loc

            deltas = {'MoveF': {'NORTH': (0,1),  'SOUTH': (0,-1), 'EAST': (1,0),  'WEST': (-1,0)},
                      'MoveB': {'NORTH': (0,-1), 'SOUTH': (0,1),  'EAST': (-1,0), 'WEST': (1,0)}  }
            new_loc = api.State(*([sum(dim) for dim in zip((self.loc.x, self.loc.y), deltas[action][self.loc.direction])] + [self.loc.direction]))

            # Load commands
            cmds = self.vehicle.commands
            #cmds.clear()

            if 0 <= new_loc.x < 6 and 0 <= new_loc.y < 4:
                print "Taking action %s to go from %s to %s\n" % (action, current_loc, new_loc)
                self.loc = new_loc
                wp = get_location_offset_meters(self.home, *get_offsets(self.grid, self.loc.x, self.loc.y))
                cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
                cmds.add(cmd)

                cmds.upload()
                time.sleep(2)
            else:
                print "Attempted to move out of grid.\n"

        elif action == "TurnCW" or action == "TurnCCW":
            res_orientations = {'NORTH': {'TurnCW': 'EAST', 'TurnCCW': 'WEST'},
                                'SOUTH': {'TurnCW': 'WEST', 'TurnCCW': 'EAST'},
                                'EAST':  {'TurnCW': 'SOUTH','TurnCCW': 'NORTH'},
                                'WEST':  {'TurnCW': 'NORTH','TurnCCW': 'SOUTH'}}
            new_loc = api.State(self.loc.x, self.loc.y, res_orientations[self.loc.direction][action])
            print "Taking action %s to go from %s to %s" % (action, self.loc, new_loc)
            self.loc = new_loc
            wp = get_location_offset_meters(self.home, *get_offsets(self.grid, self.loc.x, self.loc.y))
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 1, 90, 0, 1 if action == "TurnCW" else -1, 1, wp.lat, wp.lon, wp.alt)

        else:
            print "Invalid action"
            exit(-1)
        if len(self.actions) == 0:
            self.status_publisher.publish(self.free)


if __name__ == "__main__":
    try:
        moveDrone()
    except rospy.ROSInterruptException:
        pass
