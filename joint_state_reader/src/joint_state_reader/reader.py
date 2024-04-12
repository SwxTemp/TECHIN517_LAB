#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy 
from sensor_msgs.msg import JointState                                                                                          
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self): 
        #Initialise variable to store latest joint_state_data
        self.joint_state_data = None
        #Initialise subscriber that subsribes to jointstate
        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback)  


    #Function to fetch the latest joint state data                                                                                   
    def callback(self, data):
        self.joint_state_data = data

    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                               
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """
        #Check that the joint state data is not empty
        if self.joint_state_data is not None:
            #get the index of the current jointstate
            index = self.joint_state_data.name.index(name)
            #get the value of the jointstate based on the name
            position = self.joint_state_data.position[index]
            return position  
        return None
                                                                                            
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """          
        list_of_joint_states = []  
        #Call the getjoint method based on the length of the name
        for name in names:
            joints = self.get_joint(name)
            list_of_joint_states.append(joints)
    
        return list_of_joint_states
