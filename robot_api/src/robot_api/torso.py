#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = 'torso_controller/follow_joint_trajectory'  # Specific action server for the torso
JOINT_NAME = 'torso_lift_joint'  # The joint name for the torso

TIME_FROM_START = 5  # Duration in seconds for the action to complete

class Torso(object):
    """Torso controls the robot's torso height."""
    MIN_HEIGHT = 0.0  # Minimum height of the torso
    MAX_HEIGHT = 0.4  # Maximum height of the torso

    def __init__(self):
        # Initialize the ROS node (if not already initialized)
        if not rospy.get_node_uri():
            rospy.init_node('torso_action_client', anonymous=True)
            
        # Create an actionlib client to send trajectory commands
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height within a specified range."""
        if not self.MIN_HEIGHT <= height <= self.MAX_HEIGHT:
            rospy.logerr('Height out of range: Must be between {} and {} meters.'.format(self.MIN_HEIGHT, self.MAX_HEIGHT))
            return

        # Create a trajectory point to set the desired height
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # Create the goal using the trajectory point
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [JOINT_NAME]
        goal.trajectory.points.append(point)

        # Send the goal to the action server and wait for it to complete
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('torso_control_demo')
    torso = Torso()
    torso.set_height(0.2)  # Example usage: Set torso height to 0.2 meters
