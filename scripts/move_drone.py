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

import rospy
from std_msgs.msg import String
import math
import copy, json, time
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil

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
   return float(grid[x][y]['x'])+6, float(grid[x][y]['y'])+10, float(grid[x][y]['z'])+5.5

def init_drone(grid):
    connection_string = '127.0.0.1:14540'
    MAV_MODE_AUTO = 4

    print "Connecting"
    vehicle = connect(connection_string, wait_ready=True)

    def PX4setMode(mavMode):
        vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

    home_position_set = False

    #Create a message listener for home position fix
    @vehicle.on_message('HOME_POSITION')
    def listener(self, name, home_position):
        global home_position_set
        home_position_set = True

    home_position_set = False

    #wait for a home position lock
#    while not home_position_set:
#        print "Waiting for home position..."
#        time.sleep(1)

    print "Home Location Set %s" % vehicle.home_location

    # Display basic vehicle state
    print " Type: %s" % vehicle._vehicle_type
    print " Armed: %s" % vehicle.armed
    print " System status: %s" % vehicle.system_status.state
    print " GPS: %s" % vehicle.gps_0
    print " Alt: %s" % vehicle.location.global_relative_frame.alt

    # Change to AUTO mode
    PX4setMode(MAV_MODE_AUTO)
    time.sleep(1)

    # Load commands
    cmds = vehicle.commands
    cmds.clear()


    home = vehicle.location.global_relative_frame

    #takeoff to starting point
    wp = get_location_offset_meters(home, 0, 0, 10);
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # TODO: Move to actual starting position
    wp = get_location_offset_meters(home, *get_offsets(grid, 0, 0))
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Upload mission
    cmds.upload()
    time.sleep(2)

    # Arm vehicle
    vehicle.armed = True

    # monitor mission execution
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next+1
            print "Moving to waypoint %s" % display_seq
            nextwaypoint = vehicle.commands.next
        time.sleep(1)

    return vehicle, home


class moveDrone:
    def __init__(self):
        rospy.init_node('move_drone',anonymous = True)
        self.actions = String()
        self.loc = (0,0,"north")
        self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
        self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
        self.free = String(data = "next")
        self.rate = rospy.Rate(30)
        with open('reef.json', 'r') as f:
            self.grid = json.load(f)
        self.vehicle, self.home = init_drone(self.grid)
        print("Ready!")
        rospy.spin()

    def callback_actions(self,data):
        self.actions = data.data.split("_")
        self.rate.sleep()
        self.execute_next()

    def execute_next(self):
        action = self.actions.pop(0)
        direction = None
        if action == "MoveF" or action == "MoveB":
            current_loc = self.loc
            orientation = current_loc[2]            

            deltas = {'MoveF': {'north': (0,1),  'south': (0,-1), 'east': (1,0),  'west': (-1,0)},
                      'MoveB': {'north': (0,-1), 'south': (0,1),  'east': (-1,0), 'west': (1,0)}  }
            self.loc = tuple([sum(dim) for dim in zip(current_loc[:2], deltas[action][orientation])] + [orientation])
            x, y = self.loc[0], self.loc[1]
            # Load commands
            cmds = self.vehicle.commands
            #cmds.clear()

            wp = get_location_offset_meters(self.home, *get_offsets(self.grid, x, y))
            cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
            cmds.add(cmd)

            cmds.upload()
            time.sleep(2)

        elif action == "TurnCW" or action == "TurnCCW":
            x, y = self.loc[0], self.loc[1]
            res_orientations = {'north': {'TurnCW': 'east', 'TurnCCW': 'west'},
                                'south': {'TurnCW': 'west', 'TurnCCW': 'east'},
                                'east':  {'TurnCW': 'south','TurnCCW': 'north'},
                                'west':  {'TurnCW': 'north','TurnCCW': 'south'}}
            self.loc = (x, y, res_orientations[self.loc[2]][action])
            wp = get_location_offset_meters(self.home, *get_offsets(self.grid, x, y))
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
