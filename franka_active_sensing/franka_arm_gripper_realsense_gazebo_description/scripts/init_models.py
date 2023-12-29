#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Wrench, Pose, Twist
import cv2
import numpy as np
import depth_subscriber_segmentator
import demo_image_subscriber
import time
import random

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:
	
    object_names = []
    invariable_model_names = {'ground_plane','workcell-assembly-v2-kinnect','arm','open-cardboard-box'}
    tol = 0.00001

    

    def pause_gazebo_physics(self):
        pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        pause()
        print('gazebo paused')
        time.sleep(0.1)
	
    def unpause_gazebo_physics(self):
        unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        unpause()
        print('gazebo unpaused')
        time.sleep(0.1)
	
    		
    def init_all_models(self):
        get_world_prop=rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        try:
            world_prop = get_world_prop()
            print('object names:', world_prop.model_names)
            object_names = world_prop.model_names
            
            model_poses = []
            
            #clear the scene first
            for model_name in world_prop.model_names:
                if(model_name in self.invariable_model_names):
                    continue
                else:
                    print('to move object:', model_name)
                    
                    pose = self.get_pose(model_name)
                    model_poses.append(pose)
                    self.move_object(model_name,pose,True)
                    
               
        except rospy.ServiceException as e:
            rospy.loginfo("Get World Properties service call failed:  {0}".format(e))
		
    
    def get_pose(self,model_name):
        original_state = None
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            original_state = get_model_state(model_name, 'world')
            time.sleep(0.1)
            #self.unpause_gazebo_physics()
            #time.sleep(0.1)
            #self.pause_gazebo_physics()
            #print("Valeur de X : " + str(resp_coordinates.pose.position.x))
            #print('original_coordinates',original_state.pose)
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))    
        
        return original_state.pose
	
    def move_object(self,model_name,pose,displace):
        try:
            #displace model
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            new_state = ModelState()
            new_state.model_name = model_name
            new_state.pose = Pose()
            new_state.twist = Twist()
            new_state.reference_frame = 'world'
        
            
            new_state.pose.position.x = pose.position.x
            new_state.pose.position.y = pose.position.y
            new_state.pose.position.z = pose.position.z
            
            new_state.pose.orientation.x = pose.orientation.x
            new_state.pose.orientation.y = pose.orientation.y
            new_state.pose.orientation.z = pose.orientation.z
            new_state.pose.orientation.w = pose.orientation.w
            
            if(displace):
                new_state.pose.position.x = 0.75+0.3*random.random() 
                new_state.pose.position.z = 1.8+0.1*random.random() 
                new_state.pose.position.y = 0.5-0.15+0.3*random.random() 
				
            #print(new_state.pose.position)
            resp1 = set_model_state(new_state)
            time.sleep(0.1)
            self.unpause_gazebo_physics()
            time.sleep(1.1)
            self.pause_gazebo_physics()
            
            
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
	
	
if __name__ == '__main__':
    tuto = Tutorial()
    tuto.pause_gazebo_physics()
    tuto.init_all_models()
    tuto.pause_gazebo_physics()
    tuto.unpause_gazebo_physics()
    

#rosservice call /gazebo/get_world_properties
