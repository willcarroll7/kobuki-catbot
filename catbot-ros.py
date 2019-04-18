"""
AI Robotics - Spring 2019
Jessie Cossitt, Will Carroll

catbot-ros.py - main executable Python file for kobuki-catbot project
"""

import math
import rospy

import kobuki_msgs.msg as kmm
import geometry_msgs.msg as gmm

# Constants
STATE_SCAN_OBJECTS = 0
STATE_DIVERT_EDGE = 99

CLIFF_SENSORS = ["Left", "Center", "Right"]

# Global Variables
robot_state = STATE_SCAN_OBJECTS


def cliff_event_callback(event):
    """Triggered upon Turtlebot cliff detector change event.

    If an edge is detected, the robot enters a special edge diversion state which redirects the robot away from the edge
    and resets it into the environment scanning state."""

    global robot_state

    # Ignore undetection of cliff
    if event.state == 0:
        return

    print("*CLIFF DETECTED*")
    print("\tSensor: %s" % CLIFF_SENSORS[event.sensor])
    print("\tAborting state %s." % str(robot_state))
    print("\tEntering state DIVERT_EDGE.")

    robot_state = STATE_DIVERT_EDGE

