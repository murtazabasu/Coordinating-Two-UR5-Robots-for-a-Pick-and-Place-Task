#!/usr/bin/env python2
# Send a value to change the opening of the Robotiq gripper using an action

import argparse
from ur5_notebook.msg import Tracker2_second
import geometry_msgs.msg
import rospy, sys, numpy as np
import actionlib
import control_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

tracker = Tracker2_second()

def gripper_client(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/slave/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1  # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


def trigger(tracker):
    gripper_trigger = tracker.flag2
    print ("gripper_r1_open", gripper_trigger)
    if gripper_trigger:

        try:
            # Get the angle from the command line
            parser = argparse.ArgumentParser()
            parser.add_argument("--value", type=float, default="0.23",
                                help="Value betwewen 0.0 (open) and 0.8 (closed)")
            args = parser.parse_args()
            gripper_value = args.value
            # Start the ROS node
            
            # Set the value to the gripper
            gripper_client(gripper_value)
            
        except rospy.ROSInterruptException:
            print ("Program interrupted before completion")


    else:
        try:
            # Get the angle from the command line
            parser = argparse.ArgumentParser()
            parser.add_argument("--value", type=float, default="-0.5",
                                help="Value betwewen 0.0 (open) and 0.8 (closed)")
            args = parser.parse_args()
            gripper_value = args.value
            # Start the ROS node
            
            # Set the value to the gripper
            gripper_client(gripper_value)
            
        except rospy.ROSInterruptException:
            print ("Program interrupted before completion")


rospy.init_node('gripper_command_r1')
cxy_sub = rospy.Subscriber('/slave/cxy1', Tracker2_second, trigger, queue_size=1)
rospy.spin()

