#!/usr/bin/env python

import rospy
import control_msgs.msg
import actionlib

ACTION_NAME = '/gripper_controller/gripper_action'  # Modify as necessary
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters)
OPENED_POS = 0.10  # The position for a fully-open gripper (meters)

class Gripper(object):
    """Gripper controls the robot's gripper."""

    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.GripperCommandAction)
        # Wait for the server
        rospy.loginfo("Waiting for gripper action server...")
        self.client.wait_for_server()
        rospy.loginfo("Gripper action server found!")

    def open(self):
        """Opens the gripper."""
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS  # Set to fully open
        goal.command.max_effort = self.MIN_EFFORT  # Minimal effort
        self.client.send_goal(goal)
        rospy.loginfo("Opening gripper...")
        self.client.wait_for_result()
        rospy.loginfo("Gripper opened.")

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS  # Set to fully closed
        goal.command.max_effort = max_effort  # Use specified effort
        self.client.send_goal(goal)
        rospy.loginfo("Closing gripper...")
        self.client.wait_for_result()
        rospy.loginfo("Gripper closed.")

