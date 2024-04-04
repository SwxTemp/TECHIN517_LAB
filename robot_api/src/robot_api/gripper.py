#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal  # Import necessary message types

ACTION_NAME = 'gripper_controller/gripper_action'  # The action server name for the gripper
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).

class Gripper(object):
    """Gripper controls the robot's gripper."""
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, GripperCommandAction)
        # Wait for the action server to come up
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper."""
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS  # Set the position for the gripper to open
        goal.command.max_effort = 0  # Set max effort to 0 for opening
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS  # Set the position for the gripper to close
        goal.command.max_effort = max_effort  # Set the maximum effort for closing
        self.client.send_goal(goal)
        self.client.wait_for_result()


