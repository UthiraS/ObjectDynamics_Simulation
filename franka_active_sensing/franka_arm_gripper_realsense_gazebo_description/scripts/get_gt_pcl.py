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

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:
	
    object_names = []
    invariable_model_names = {'ground_plane','workcell-assembly-v2-kinnect','arm','open-cardboard-box'}
    #invariable_model_names = {'ground_plane','workcell-assembly-v2-kinnect','arm','open-cardboard-box','bag','bolt','nut','RustPipe','Wrench','needlenose','Wrench','barrel','fluorescent-strip','pallet'}
    tol = 0.00001

    

    def pause_gazebo_physics(self):
		pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
		pause()
		print('gazebo paused')
		time.sleep(0.3)
	
    def unpause_gazebo_physics(self):
		unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
		unpause()
		print('gazebo unpaused')
		time.sleep(0.3)
	
    def get_depthmap(self):
        subs_obj = depth_subscriber_segmentator.Depth_subs()
        while not rospy.is_shutdown():
            time.sleep(0.1)
            if subs_obj.depth is not None:
                #time.sleep(0.3)
                break;
        #return subs_obj.depth
        return subs_obj
    
    		
    def get_object_gt_depth(self):
        get_world_prop=rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        try:
            world_prop = get_world_prop()
            print('object names:', world_prop.model_names)
            object_names = world_prop.model_names
            
            model_poses = []
            
            
            #Get initial image & depthmap
            
            kinect_subs = self.get_depthmap()
            #image_sub = self.get_image()
            depthmap_init = kinect_subs.depth
            image_init = kinect_subs.image
            depthmap_init_normalized = cv2.normalize(depthmap_init, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            
            cv2.imwrite('results/depthmap_init.png',depthmap_init_normalized)
            cv2.imwrite('results/image_init.png',image_init)
            
            #clear the scene first
            for model_name in world_prop.model_names:
                if(model_name in self.invariable_model_names):
					continue
                else:
                    print('to move object:', model_name)
                    
                    pose = self.get_pose(model_name)
                    model_poses.append(pose)
                    self.move_object(model_name,pose,True)
                    
                
            
            #Get depthmap of bacground, we will use it for subtraction
            depthmap_bg = kinect_subs.depth
            image_bg = kinect_subs.image
            depthmap_bg_normalized = cv2.normalize(depthmap_bg, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            #print depthmap_bg
            #cv2.imshow("depth_in_main", depthmap_bg)
            #keystroke = cv2.waitKey(500)
            cv2.imwrite('interim/depthmap_bg.png',depthmap_bg_normalized)
            cv2.imwrite('interim/image_bg.png',image_bg)
        
            
            depthmaps = []
            
            #Bring each object to original pose and get depthmap 
            i = 0
            for model_name in world_prop.model_names:
                if(model_name in self.invariable_model_names):
					continue
                else:
                    print('to move object:', model_name)
                    self.move_object(model_name,model_poses[i],False)
                    fname = 'interim/depthmap_%d.png' % (i)
                    image_fname = 'interim/image_%d.png' % (i)
                    fname_ori = 'interim/depthmap_ori_%d.png' % (i)
                    
                    
                    time.sleep(0.3)
                    depthmap_curr = kinect_subs.depth
                    depth_image_normalized = cv2.normalize(depthmap_curr, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
                    cv2.imwrite(fname_ori,depth_image_normalized)
                    depthmap_curr = (abs(depthmap_bg.astype(float) - depthmap_curr.astype(float))*255).astype(np.uint8)
                    depthmap_curr_normalized = cv2.normalize(depthmap_curr, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
                    #image_curr = (abs(cv2.cvtColor(image_bg,cv2.COLOR_BGR2GRAY).astype(float) - cv2.cvtColor(self.get_image(),cv2.COLOR_BGR2GRAY).astype(float))).astype(np.uint8)
                    #image_curr = self.get_image()
                    image_curr = kinect_subs.image
                    #mask_image  = np.ceil(np.mean(abs(image_curr.astype(float)-image_bg.astype(float)),axis = 2)).astype(np.uint8)
                    #mask_image  = image_curr
                    #ret,mask_image_th = cv2.threshold(mask_image,0,1,cv2.THRESH_BINARY)
                    
                    #depthmap_curr = (np.multiply(depthmap_curr.astype(float),mask_image_th.astype(float))).astype(np.uint8)
                    #cv2.imwrite(fname,self.get_depthmap())
                    
                    cv2.imwrite(image_fname,depthmap_curr_normalized)
                    cv2.imwrite(fname,depthmap_curr)
                    
                    depthmaps.append(depthmap_curr)
                    self.move_object(model_name,model_poses[i],True)
                    #cv2.imshow("depth_in_main", depthmap_curr)
                    #keystroke = cv2.waitKey(500)
                    i += 1
            
            
            #Reconstuct original scene
            i = 0
            for model_name in world_prop.model_names:
                if(model_name in self.invariable_model_names):
					continue
                else:
                    print('to reset object:', model_name)
                    self.move_object(model_name,model_poses[i],False)
                    i += 1
            
            #Merge the gt Maps
            
            gt_image = np.zeros((depthmap_curr.shape[0],depthmap_curr.shape[1]), np.uint8)
            
            depthmaps_arr = np.squeeze(depthmaps)
            print depthmaps_arr.shape
            
            gt_image =np.argmax(depthmaps_arr,axis = 0)+1
            
            
            mask_image  = np.ceil(np.mean(depthmaps_arr.astype(float),axis = 0)).astype(np.uint8)
            ret,mask_image_th = cv2.threshold(mask_image,0,1,cv2.THRESH_BINARY)
            cv2.imwrite('interim/gt_image.png',gt_image)
            cv2.imwrite('interim/mask_image_th.png',200*mask_image_th)
            
            gt_image1 = (np.multiply(gt_image.astype(float),mask_image_th.astype(float))).astype(np.uint8)
            
            
            
            gt_normalized = cv2.normalize(gt_image1, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            cv2.imshow("depth_in_main", 10*mask_image)
            keystroke = cv2.waitKey(2000)   
            
            gt_color = cv2.applyColorMap(gt_normalized, cv2.COLORMAP_JET)
            cv2.imshow("depth_in_main", gt_color)
            keystroke = cv2.waitKey(2000)
            
            cv2.imwrite('results/segmentation_gray.png',10*gt_image1)
            cv2.imwrite('results/segmentation_final.png',gt_color)
               
        except rospy.ServiceException as e:
            rospy.loginfo("Get World Properties service call failed:  {0}".format(e))
		
    
    def get_pose(self,model_name):
        original_state = None
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            original_state = get_model_state(model_name, 'world')
            time.sleep(0.1)
            
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
                new_state.pose.position.x += 10
				
            
            resp1 = set_model_state(new_state)
            time.sleep(0.3)
            
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
	
	
if __name__ == '__main__':
    tuto = Tutorial()
    tuto.pause_gazebo_physics()
    tuto.get_object_gt_depth()
    tuto.unpause_gazebo_physics()
    

#rosservice call /gazebo/get_world_properties
