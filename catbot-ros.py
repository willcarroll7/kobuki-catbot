"""
AI Robotics - Spring 2019
Jessie Cossitt, Will Carroll

catbot-ros.py - main executable Python file for kobuki-catbot project
"""

import math
import rospy

import kobuki_msgs.msg as kmm
import geometry_msgs.msg as gmm

# FSM States
STATE_SCAN_OBJECTS = 0
STATE_PREPARE_YEET = 10
STATE_YEET = 11
STATE_ALIGN_OBJECT = 12
STATE_APPROACH_OBJECT = 13
STATE_DIVERT_EDGE = 98
STATE_TURN_180 = 99

# Constants
CLIFF_SENSORS = ["Left", "Center", "Right"]
NAV_TURN_SPEED = math.pi / 4.0
YEET_SPEED = math.pi
FORWARD_SPEED = 0.1
ENC_CLICKS_PER_DEG = 24.444444444
ENC_CLICKS_SLACK = ENC_CLICKS_PER_DEG * 3.0  # 3.0 degrees of slack

# Global Variables
robot_state = STATE_SCAN_OBJECTS
cliff_sensors = [False, False, False]
left_encoder = right_encoder = 0
object_found = object_visible = True


def cliff_event_callback(event):
    """Triggered upon Turtlebot cliff detector change event.

    If an edge is detected, the robot enters a special edge diversion state which redirects the robot away from the edge
    and resets it into the environment scanning state."""

    global cliff_sensors, robot_state

    cliff_sensors[event.sensor] = bool(event.state)

    # Ignore undetection of cliff
    if event.state == 0 or robot_state == STATE_TURN_180:
        return

    print("*CLIFF DETECTED*")
    print("\tSensor: %s" % CLIFF_SENSORS[event.sensor])
    print("\tAborting state %s." % str(robot_state))
    print("\tEntering state DIVERT_EDGE.")

    robot_state = STATE_DIVERT_EDGE


def sensor_core_callback(msg):
    """Triggered upon Turtlebot sensor data received."""

    global left_encoder, right_encoder

    left_encoder = msg.left_encoder
    right_encoder = msg.right_encoder


# Setup application and run FSM loop
if __name__ == "__main__":
    rospy.init_node("catbot",  anonymous=True)
    rospy.Subscriber("/mobile_base/events/cliff", kmm.CliffEvent, cliff_event_callback)
    rospy.Subscriber("/mobile_base/sensors/core", kmm.SensorState, sensor_core_callback)
    navi = rospy.Publisher("/cmd_vel_mux/input/navi", gmm.Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    # State helper variables
    left_encoder_target = right_encoder_target = 0
    twist = gmm.Twist()

    while not rospy.is_shutdown():
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if robot_state == STATE_SCAN_OBJECTS:
            # Look for objects and spin until one is found
            if object_found:
                robot_state = STATE_ALIGN_OBJECT
                
            # else keep turning
            else:
                twist.linear.x = 0.0
                twist.angular.z = NAV_TURN_SPEED
            
        elif robot_state == STATE_PREPARE_YEET:
            # set encoder targets for 360 degree turn
            left_encoder_target = left_encoder - (ENC_CLICKS_PER_DEG * 360.0)
            right_encoder_target = right_encoder + (ENC_CLICKS_PER_DEG * 360.0)
            robot_state = STATE_YEET
            
        elif robot_state == STATE_YEET:
            # Spin quickly 360 degrees to yeet enemy object
            if abs(left_encoder - left_encoder_target) < ENC_CLICKS_SLACK or abs(
                    right_encoder - right_encoder_target) < ENC_CLICKS_SLACK:
                robot_state = STATE_SCAN_OBJECTS
            else:
                twist.linear.x = 0.0
                twist.angular.z = YEET_SPEED
            
        elif robot_state == STATE_ALIGN_OBJECT:
            # Align so scanned object is centered with the image
            robot_state = STATE_APPROACH_OBJECT
            
        elif robot_state == STATE_APPROACH_OBJECT:
            # Go toward object until it is close enough not to be seen
            if object_visible:  # TODO
                twist.linear.x = FORWARD_SPEED
                twist.angular.z = 0.0
                
            # else YEET
            else:
                robot_state = STATE_PREPARE_YEET

        elif robot_state == STATE_DIVERT_EDGE:
            # Move away from edge
            twist.linear.x = -FORWARD_SPEED / 2.0
            twist.angular.z = 0.0
            navi.publish(twist)  # Go ahead and publish command because conditionals are slow and we need to stop ASAP
            
            if not cliff_sensors[0] and not cliff_sensors[1] and not cliff_sensors[2]:
                # Set the encoder targets for a 180 degree turn
                left_encoder_target = left_encoder - (ENC_CLICKS_PER_DEG * 180.0)
                right_encoder_target = right_encoder + (ENC_CLICKS_PER_DEG * 180.0)
                robot_state = STATE_TURN_180

        elif robot_state == STATE_TURN_180:
            # Return to STATE_SCAN_OBJECTS if we've finished turning around
            if abs(left_encoder - left_encoder_target) < ENC_CLICKS_SLACK or abs(
                    right_encoder - right_encoder_target) < ENC_CLICKS_SLACK:
                robot_state = STATE_SCAN_OBJECTS

            # Else keep turning
            else:
                twist.angular.z = NAV_TURN_SPEED

        navi.publish(twist)
        rate.sleep()
