#!/usr/bin/env python

# import rospy
# import pickle
# from geometry_msgs.msg import PoseStamped
# from map_annotator.msg import UserAction, PoseNames
# from interactive_markers.interactive_marker_server import *
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

# class MapAnnotatorServer:
#     def __init__(self):
#         self.pose_db_name = rospy.get_param('~pose_db_name', 'our_pose_db_name.db')
#         self.poses = self.load_poses()
#         self.server = InteractiveMarkerServer("map_poses")

#         self.pub_pose_names = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
#         self.sub_user_actions = rospy.Subscriber('/map_annotator/user_actions', UserAction, self.handle_user_action)

#         # Add this line to create the publisher for sending goal poses
#         self.pub_goal_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

#         self.update_pose_list()

#     def load_poses(self):
#         try:
#             with open(self.pose_db_name, 'rb') as f:
#                 return pickle.load(f)
#         except IOError:
#             rospy.loginfo("No existing pose database found. Creating a new one.")
#             return {}

#     def save_poses(self):
#         with open(self.pose_db_name, 'wb') as f:
#             pickle.dump(self.poses, f)

#     def handle_user_action(self, msg):
#         if msg.command == UserAction.CREATE:
#             self.create_pose(msg.name)
#         elif msg.command == UserAction.DELETE:
#             self.delete_pose(msg.name)
#         elif msg.command == UserAction.GOTO:
#             self.goto_pose(msg.name)
#         elif msg.command == "rename":  # Optional: Uncomment if implementing rename functionality
#             self.rename_pose(msg.name, msg.updated_name)

#     def create_pose(self, name):
#         if name not in self.poses:
#             self.poses[name] = {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]}  # Default pose
#             self.update_pose_list()
#             self.add_interactive_marker(name)

#     def delete_pose(self, name):
#         if name in self.poses:
#             del self.poses[name]
#             self.update_pose_list()
#             self.server.erase(name)
#             self.server.applyChanges()

#     # def goto_pose(self, name):
#     #     rospy.loginfo(f"Going to pose: {name}")

#     def rename_pose(self, old_name, new_name):
#         if old_name in self.poses and new_name not in self.poses:
#             self.poses[new_name] = self.poses.pop(old_name)
#             self.update_pose_list()
#             # Rename interactive marker as needed

#     def update_pose_list(self):
#         pose_names_msg = PoseNames()
#         pose_names_msg.names = list(self.poses.keys())
#         self.pub_pose_names.publish(pose_names_msg)

#     def add_interactive_marker(self, name):
#         int_marker = InteractiveMarker()
#         int_marker.header.frame_id = "map"  # Use the map frame for the interactive marker
#         int_marker.name = name
#         int_marker.description = name

#         # Create the arrow marker
#         arrow_marker = Marker()
#         arrow_marker.type = Marker.ARROW
#         arrow_marker.scale.x = 0.5
#         arrow_marker.scale.y = 0.1
#         arrow_marker.scale.z = 0.1
#         arrow_marker.color.r = 0.0
#         arrow_marker.color.g = 1.0
#         arrow_marker.color.b = 0.0
#         arrow_marker.color.a = 1.0

#         # Create the circular marker
#         circle_marker = Marker()
#         circle_marker.type = Marker.CYLINDER
#         circle_marker.scale.x = 0.5
#         circle_marker.scale.y = 0.5
#         circle_marker.scale.z = 0.02
#         circle_marker.color.r = 0.0
#         circle_marker.color.g = 0.0
#         circle_marker.color.b = 1.0
#         circle_marker.color.a = 0.5

#         # Create the control for moving the marker in the XY plane
#         move_control = InteractiveMarkerControl()
#         move_control.orientation.w = 1.0
#         move_control.orientation.x = 0.0
#         move_control.orientation.y = 1.0
#         move_control.orientation.z = 0.0
#         move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
#         move_control.always_visible = True
#         move_control.markers.append(arrow_marker)
#         move_control.markers.append(circle_marker)
#         int_marker.controls.append(move_control)

#         # Create the control for rotating the marker
#         rotate_control = InteractiveMarkerControl()
#         rotate_control.orientation.w = 1.0
#         rotate_control.orientation.x = 0.0
#         rotate_control.orientation.y = 1.0
#         rotate_control.orientation.z = 0.0
#         rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
#         rotate_control.always_visible = True
#         int_marker.controls.append(rotate_control)

#         # Add the text marker to display the pose name
#         text_marker = Marker()
#         text_marker.type = Marker.TEXT_VIEW_FACING
#         text_marker.text = name
#         text_marker.scale.z = 0.1
#         text_marker.color.r = 1.0
#         text_marker.color.g = 1.0
#         text_marker.color.b = 1.0
#         text_marker.color.a = 1.0
#         text_marker.pose.position.z = 0.2  # Adjust the height of the text marker

#         text_control = InteractiveMarkerControl()
#         text_control.interaction_mode = InteractiveMarkerControl.NONE
#         text_control.always_visible = True
#         text_control.markers.append(text_marker)
#         int_marker.controls.append(text_control)
	
#     def goto_pose(self, name):
#         if name in self.poses:
#             pose = self.poses[name]
#             goal_pose = PoseStamped()
#             goal_pose.header.frame_id = "map"
#             goal_pose.pose.position.x = pose['position'][0]
#             goal_pose.pose.position.y = pose['position'][1]
#             goal_pose.pose.position.z = pose['position'][2]
#             goal_pose.pose.orientation.x = pose['orientation'][0]
#             goal_pose.pose.orientation.y = pose['orientation'][1]
#             goal_pose.pose.orientation.z = pose['orientation'][2]
#             goal_pose.pose.orientation.w = pose['orientation'][3]

#             # Send the goal pose to the navigation stack
#             self.pub_goal_pose.publish(goal_pose)
#             rospy.loginfo(f"Sending robot to pose: {name}")
#         else:
#             rospy.logwarn(f"Pose '{name}' not found.")

#     def process_feedback(self, feedback):
#         p = feedback.pose.position
#         o = feedback.pose.orientation
#         self.poses[feedback.marker_name] = {'position': [p.x, p.y, p.z], 'orientation': [o.x, o.y, o.z, o.w]}
#         self.save_poses()

#     def on_shutdown(self):
#         self.save_poses()

    

# if __name__ == '__main__':
#     rospy.init_node('map_annotator_server')
#     server = MapAnnotatorServer()
#     rospy.on_shutdown(server.on_shutdown)
#     rospy.spin()


#!/usr/bin/env python

import rospy
import pickle
from std_msgs.msg import String
from map_annotator.msg import UserAction, PoseNames
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped

class MapAnnotatorServer:
    def __init__(self):
        self.pose_db_name = rospy.get_param('~pose_db_name', 'our_pose_db_name.db')
        self.poses = self.load_poses()
        self.server = InteractiveMarkerServer("map_poses")

        self.pub_pose_names = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self.sub_user_actions = rospy.Subscriber('/map_annotator/user_actions', UserAction, self.handle_user_action)
        self.pub_goal_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.update_pose_list()

    def load_poses(self):
        try:
            with open(self.pose_db_name, 'rb') as f:
                return pickle.load(f)
        except IOError:
            rospy.loginfo("No existing pose database found. Creating a new one.")
            return {}

    def save_poses(self):
        with open(self.pose_db_name, 'wb') as f:
            pickle.dump(self.poses, f)

    def handle_user_action(self, msg):
        if msg.command == UserAction.CREATE:
            self.create_pose(msg.name)
        elif msg.command == UserAction.DELETE:
            self.delete_pose(msg.name)
        elif msg.command == UserAction.GOTO:
            self.goto_pose(msg.name)

    def create_pose(self, name):
        if name not in self.poses:
            self.poses[name] = {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]}  # Default pose
            self.update_pose_list()
            self.add_interactive_marker(name)

    def delete_pose(self, name):
        if name in self.poses:
            del self.poses[name]
            self.update_pose_list()
            self.server.erase(name)
            self.server.applyChanges()

    def goto_pose(self, name):
        if name in self.poses:
            pose = self.poses[name]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = pose['position'][0]
            goal_pose.pose.position.y = pose['position'][1]
            goal_pose.pose.position.z = pose['position'][2]
            goal_pose.pose.orientation.x = pose['orientation'][0]
            goal_pose.pose.orientation.y = pose['orientation'][1]
            goal_pose.pose.orientation.z = pose['orientation'][2]
            goal_pose.pose.orientation.w = pose['orientation'][3]
            rospy.loginfo(f"goal pose: {goal_pose}")
            # Send the goal pose to the navigation stack
            self.pub_goal_pose.publish(goal_pose)
            rospy.loginfo(f"Going to pose: {name}")
        else:
            rospy.logwarn(f"Pose '{name}' not found.")


    def update_pose_list(self):
        pose_names_msg = PoseNames()
        pose_names_msg.names = list(self.poses.keys())
        self.pub_pose_names.publish(pose_names_msg)

    def add_interactive_marker(self, name):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        control.markers.append(arrow_marker)

        circle_marker = Marker()
        circle_marker.type = Marker.CYLINDER
        circle_marker.scale.x = 0.5
        circle_marker.scale.y = 0.5
        circle_marker.scale.z = 0.02
        circle_marker.color.r = 0.0
        circle_marker.color.g = 0.0
        circle_marker.color.b = 1.0
        circle_marker.color.a = 0.5
        control.markers.append(circle_marker)

        int_marker.controls.append(control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.always_visible = True
        int_marker.controls.append(rotate_control)

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = name
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.pose.position.z = 0.2

        text_control = InteractiveMarkerControl()
        text_control.interaction_mode = InteractiveMarkerControl.NONE
        text_control.always_visible = True
        text_control.markers.append(text_marker)
        int_marker.controls.append(text_control)

        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        p = feedback.pose.position
        o = feedback.pose.orientation
        self.poses[feedback.marker_name] = {'position': [p.x, p.y, p.z], 'orientation': [o.x, o.y, o.z, o.w]}
        self.save_poses()

    def on_shutdown(self):
        self.save_poses()

if __name__ == '__main__':
    rospy.init_node('map_annotator_server')
    server = MapAnnotatorServer()
    rospy.on_shutdown(server.on_shutdown)
    rospy.spin()