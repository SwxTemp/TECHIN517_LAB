import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .arm_joints import ArmJoints  # Make sure this import works in your package structure

class Arm(object):
    """Arm controls the robot's arm."""

    def __init__(self):
        # Adjust the action server name based on your robot setup
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller action server...")
        self.client.wait_for_server()
        rospy.loginfo("Arm controller action server found.")

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for the arm.
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ArmJoints.names()
       
        point = JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start = rospy.Duration(5.0)  # 5 seconds to reach the target
        
        goal.trajectory.points.append(point)
       
        rospy.loginfo("Sending goal to the arm controller...")
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(6.0))  # Wait a bit longer than the trajectory time
       
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arm movement successful.")
        else:
            rospy.logerr("Arm movement failed.")



