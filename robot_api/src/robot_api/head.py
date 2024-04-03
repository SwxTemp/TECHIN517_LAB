#!/usr/bin/env python

import math
import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # Example action name for look_at
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # Example for pan/tilt
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
EYES_JOINT = '/robot/eyes_controller/follow_joint_trajectory'
class Head(object):
    """Head controls the Fetch's head."""

    MIN_PAN = -math.pi  # Example values, adjust as necessary
    MAX_PAN = math.pi
    MIN_TILT = -math.pi / 4
    MAX_TILT = math.pi / 4

    def __init__(self):
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head control action servers...")
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
        rospy.loginfo("Found action servers.")

    def look_at(self, frame_id, x, y, z):
        goal = PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1)
        self.look_at_client.send_goal(goal)
        self.look_at_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        if pan < self.MIN_PAN or pan > self.MAX_PAN or tilt < self.MIN_TILT or tilt > self.MAX_TILT:
            rospy.logerr('Requested pan/tilt angles are out of bounds.')
            return
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        trajectory.points = [point]
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self.pan_tilt_client.send_goal(goal)
        self.pan_tilt_client.wait_for_result()

PAN_TILT_TIME = 2.5  # Adjust the movement duration as needed
