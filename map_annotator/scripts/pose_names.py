# # Python pseudocode for publishing pose names
# import rospy
# from std_msgs.msg import String
# from map_annotator.msg import PoseNames  # Assuming PoseNames is a custom message type holding an array of strings

# def publish_pose_names():
#     rospy.init_node('pose_names_publisher')
#     pub = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
#     pose_names = PoseNames()
#     # TODO: Load pose names from the file or database
#     pose_names.names = # Example pose names
#     pub.publish(pose_names)

# if __name__ == '__main__':
#     while True:
#         rospy.sleep(1)
#         publish_pose_names()
#     rospy.spin()
