import socket
import select
import struct
import time
import os
import random
import threading
import argparse
import matplotlib.pyplot as plt
import scipy as sc
from scipy import ndimage
import cv2
import csv
from collections import namedtuple
import numpy as np
import utils
import transformations
from simulation import vrep

class Robot(object):
    def __init__(self, is_sim, obj_mesh_dir, num_obj, workspace_limits, test_preset_file, heightmap_resolution,             
                 is_testing, test_preset_cases):

        self.is_sim = is_sim
        self.workspace_limits = workspace_limits

        # If in simulation...
        if self.is_sim:

            # Define colors for object meshes (Tableau palette)
            self.color_space = np.asarray([[78.0, 121.0, 167.0], # blue
                                           [89.0, 161.0, 79.0], # green
                                           [156, 117, 95], # brown
                                           [242, 142, 43], # orange
                                           [237.0, 201.0, 72.0], # yellow
                                           [186, 176, 172], # gray
                                           [255.0, 87.0, 89.0], # red
                                           [176, 122, 161], # purple
                                           [118, 183, 178], # cyan
                                           [255, 157, 167]])/255.0 #pink 

            # Read files in object mesh directory 
            self.obj_mesh_dir = obj_mesh_dir
            self.num_obj = num_obj
            self.mesh_list = os.listdir(self.obj_mesh_dir)

            # Randomly choose objects to add to scene
            self.obj_mesh_ind = np.random.randint(0, len(self.mesh_list), size=self.num_obj)
            self.obj_mesh_color = self.color_space[np.asarray(range(self.num_obj)) % 10, :]

            # Make sure to have the server side running in V-REP: 
            # in a child script of a V-REP scene, add following command
            # to be executed just once, at simulation start:
            #
            # simExtRemoteApiStart(19999)
            #
            # then start simulation, and run this program.
            #
            # IMPORTANT: for each successful call to simxStart, there
            # should be a corresponding call to simxFinish at the end!

            # MODIFY remoteApiConnections.txt 

            # Connect to simulator
            vrep.simxFinish(-1) # Just in case, close all opened connections
            self.sim_client = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to V-REP on port 19997
            if self.sim_client == -1:
                print('Failed to connect to simulation (V-REP remote API server). Exiting.')
                exit()
            else:
                print('Connected to simulation.')
                self.restart_sim()

            self.is_testing = is_testing
            self.test_preset_cases = test_preset_cases
            self.test_preset_file = test_preset_file

            # Setup virtual camera in simulation
            self.setup_sim_camera()

            # If testing, read object meshes and poses from test case file
            if self.is_testing and self.test_preset_cases:
                file = open(self.test_preset_file, 'r')
                file_content = file.readlines() 
                #with open(self.test_preset_file) as f:
                #file_content = [line for line in csv.reader(f)]
                self.test_obj_mesh_files = []
                self.test_obj_mesh_colors = []
                self.test_obj_positions = []
                self.test_obj_orientations = []
                for object_idx in range(self.num_obj):
                    file_content_curr_object = file_content[object_idx].split()
                    self.test_obj_mesh_files.append(os.path.join(self.obj_mesh_dir,file_content_curr_object[0]))
                    self.test_obj_mesh_colors.append([float(file_content_curr_object[1]),float(file_content_curr_object[2]),float(file_content_curr_object[3])])
                    self.test_obj_positions.append([float(file_content_curr_object[4]),float(file_content_curr_object[5]),float(file_content_curr_object[6])])
                    self.test_obj_orientations.append([float(file_content_curr_object[7]),float(file_content_curr_object[8]),float(file_content_curr_object[9])])
                file.close()
                self.obj_mesh_color = np.asarray(self.test_obj_mesh_colors)
                color_img, depth_img = self.get_camera_data()
                depth_img = depth_img * self.cam_depth_scale
                self.color_heightmap, self.depth_heightmap = utils.get_heightmap(color_img, depth_img, self.cam_intrinsics,self.cam_pose, workspace_limits, heightmap_resolution)
            # Add objects to simulation environment
            self.add_objects()
 
    def add_objects(self):

        # Add each object to robot workspace at x,y location and orientation (random or pre-loaded)
        self.object_handles = []
        sim_obj_handles = []
        for object_idx in range(len(self.obj_mesh_ind)):
            curr_mesh_file = os.path.join(self.obj_mesh_dir, self.mesh_list[self.obj_mesh_ind[object_idx]])
            if self.is_testing and self.test_preset_cases:
                curr_mesh_file = self.test_obj_mesh_files[object_idx]
            curr_shape_name = 'shape_%02d' % object_idx
            drop_x = (self.workspace_limits[0][1] - self.workspace_limits[0][0] - 0.2) * np.random.random_sample() + self.workspace_limits[0][0] +0.1+(0.05*object_idx)
            drop_y = (self.workspace_limits[1][1] - self.workspace_limits[1][0] - 0.2) * np.random.random_sample() + self.workspace_limits[1][0] +0.1+(0.05*object_idx)
            object_position = [drop_x, drop_y, 0.15]
            object_orientation = [2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample()]
            if self.is_testing and self.test_preset_cases:
                object_position = [self.test_obj_positions[object_idx][0], self.test_obj_positions[object_idx][1], self.test_obj_positions[object_idx][2]]
                object_orientation = [self.test_obj_orientations[object_idx][0], self.test_obj_orientations[object_idx][1], self.test_obj_orientations[object_idx][2]]
            object_color = [self.obj_mesh_color[object_idx][0], self.obj_mesh_color[object_idx][1], self.obj_mesh_color[object_idx][2]]
            ret_resp,ret_ints,ret_floats,ret_strings,ret_buffer = vrep.simxCallScriptFunction(self.sim_client, 'remoteApiCommandServer',vrep.sim_scripttype_childscript,'importShape',[0,0,255,0], object_position + object_orientation + object_color, [curr_mesh_file, curr_shape_name], bytearray(), vrep.simx_opmode_blocking)
            if ret_resp == 8:
                print('Failed to add new objects to simulation. Please restart.')
                exit()
            curr_shape_handle = ret_ints[0]
            self.object_handles.append(curr_shape_handle)
            sim_ret=vrep.simxSetObjectFloatParameter(self.sim_client,curr_shape_handle, 3005,0.0009, vrep.simx_opmode_blocking)
            #sim.shapefloatparam_mass
            if not (self.is_testing and self.test_preset_cases):
                time.sleep(2)
        self.prev_obj_positions = []
        self.obj_positions = []
    
    def setup_sim_camera(self):

        # Get handle to camera
        sim_ret, self.cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_persp', vrep.simx_opmode_blocking)

        # Get camera pose and intrinsics in simulation
        sim_ret, cam_position = vrep.simxGetObjectPosition(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, cam_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking)
        cam_trans = np.eye(4,4)
        cam_trans[0:3,3] = np.asarray(cam_position)
        cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
        cam_rotm = np.eye(4,4)
        cam_rotm[0:3,0:3] = np.linalg.inv(utils.euler2rotm(cam_orientation))
        self.cam_pose = np.dot(cam_trans, cam_rotm) # Compute rigid transformation representating camera pose
        self.cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
        self.cam_depth_scale = 1

        # Get background image
        self.bg_color_img, self.bg_depth_img = self.get_camera_data()
        self.bg_depth_img = self.bg_depth_img * self.cam_depth_scale

    def restart_sim(self):

        sim_ret, self.target_handle = vrep.simxGetObjectHandle(self.sim_client,'panda_targetTip',vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.target_handle, -1, (-0.5,0,0.3), vrep.simx_opmode_blocking)
        vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
        time.sleep(1)
        sim_ret, self.tip_handle = vrep.simxGetObjectHandle(self.sim_client, 'panda_tip', vrep.simx_opmode_blocking)
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.tip_handle, -1, vrep.simx_opmode_blocking)
        while gripper_position[2] > 0.4: # V-REP bug requiring multiple starts and stops to restart
            vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
            time.sleep(1)
            sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.tip_handle, -1, vrep.simx_opmode_blocking)


    def check_sim(self):

        # Check if simulation is stable by checking if gripper is within workspace
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.tip_handle, -1, vrep.simx_opmode_blocking)
        sim_ok = gripper_position[0] > self.workspace_limits[0][0] -0.5  and gripper_position[0] < self.workspace_limits[0][1] +0.5  and gripper_position[1] > self.workspace_limits[1][0] -0.5 and gripper_position[1] < self.workspace_limits[1][1] +0.5 and gripper_position[2] > self.workspace_limits[2][0]-0.5 and gripper_position[2] < self.workspace_limits[2][1]+0.5
        if not sim_ok:
            print('Simulation unstable. Restarting environment.')
            self.restart_sim()
            self.add_objects()


    def get_task_score(self):

        key_positions = np.asarray([[-0.625, 0.125, 0.0], # red
                                    [-0.625, -0.125, 0.0], # blue
                                    [-0.375, 0.125, 0.0], # green
                                    [-0.375, -0.125, 0.0]]) #yellow

        obj_positions = np.asarray(self.get_obj_positions())
        obj_positions.shape = (1, obj_positions.shape[0], obj_positions.shape[1])
        obj_positions = np.tile(obj_positions, (key_positions.shape[0], 1, 1))

        key_positions.shape = (key_positions.shape[0], 1, key_positions.shape[1])
        key_positions = np.tile(key_positions, (1 ,obj_positions.shape[1] ,1))

        key_dist = np.sqrt(np.sum(np.power(obj_positions - key_positions, 2), axis=2))
        key_nn_idx = np.argmin(key_dist, axis=0)

        return np.sum(key_nn_idx == np.asarray(range(self.num_obj)) % 4)


    def check_goal_reached(self):

        goal_reached = self.get_task_score() == self.num_obj
        return goal_reached


    # def stop_sim(self):
    #     if self.is_sim:
    #         # Now send some data to V-REP in a non-blocking fashion:
    #         # vrep.simxAddStatusbarMessage(sim_client,'Hello V-REP!',vrep.simx_opmode_oneshot)

    #         # # Start the simulation
    #         # vrep.simxStartSimulation(sim_client,vrep.simx_opmode_oneshot_wait)

    #         # # Stop simulation:
    #         # vrep.simxStopSimulation(sim_client,vrep.simx_opmode_oneshot_wait)

    #         # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #         vrep.simxGetPingTime(self.sim_client)

    #         # Now close the connection to V-REP:
    #         vrep.simxFinish(self.sim_client)
    def random_obj(self):
       
        object_handle=random.choice(self.object_handles)
        sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
        #if
        
        return object_position

    def object_pose(self,position,orientation):
        obj_pose = transformations.euler_matrix(orientation[0], orientation[1],orientation[2])
        obj_pose[:3,-1] = np.asarray(position)
        return obj_pose

    def get_obj_positions(self):

        obj_positions = []
        for object_handle in self.object_handles:
            sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
            obj_positions.append(object_position)
        
        return obj_positions

    def get_obj_positions_and_orientations(self):

        obj_positions = []
        obj_orientations = []
        for object_handle in self.object_handles:
            sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
            sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
            obj_positions.append(object_position)
            obj_orientations.append(object_orientation)

        return obj_positions, obj_orientations

    def init_tower(self):
        temp_objects=self.object_handles
        #print(temp_objects)
        object_handle=setw=random.choice(temp_objects)
        k=[]
        #vrep.simxSetObjectPosition(self.sim_client, object_handle, -1, (-0.65,-0.1,0), vrep.simx_opmode_blocking)
        k.append(object_handle)
        u=object_handle
        sim_ret, object_position1 = vrep.simxGetObjectPosition(self.sim_client,object_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
        
        i=1        
        for ob in temp_objects:
          
          if ob not in k:
             
              sim_ret, obj_pos = vrep.simxGetObjectPosition(self.sim_client, ob, -1, vrep.simx_opmode_blocking)  
               
              l=obj_pos[2]
              angle= np.deg2rad(l*(360.0/16))             
              print("lifting box "+str(i))
              #print(i)
              #print(ob)
              sim_ret,z=vrep.simxGetObjectFloatParameter(self.sim_client,u,20,vrep.simx_opmode_blocking)
              #object_position[0]=object_position1[0]
              #object_position[1]=object_position1[1]
              object_position1[2]+=z+0.05
              
              s=True
              while(s is not False):            
                s=self.grasp(obj_pos,angle,self.workspace_limits,object_position1,object_orientation,1)              
              print("Grasp Successful :"+str(s))              
              i=i+1
              time.sleep(3)
              u=ob
              k.append(ob)
                   
          time.sleep(0.03)     
        print("!")
       
    def reposition_objects(self,workspace_limits):

        # Move gripper out of the way
        #self.move_to([-0.1, 0, 0.3], None)
        # sim_ret, target_handle = vrep.simxGetObjectHandle(self.sim_client,'panda_targetTip',vrep.simx_opmode_blocking)
        # vrep.simxSetObjectPosition(self.sim_client, target_handle, -1, (-0.5,0,0.3), vrep.simx_opmode_blocking)
        # time.sleep(1)
        object_idx=0
        for object_handle in self.object_handles:

            # Drop object at random x,y location and random orientation in robot workspace
            drop_x = (workspace_limits[0][1] - workspace_limits[0][0] - 0.2) * np.random.random_sample() + workspace_limits[0][0] + 0.1#+(0.05*object_idx)
            drop_y = (workspace_limits[1][1] -workspace_limits[1][0] - 0.2) * np.random.random_sample() + workspace_limits[1][0] + 0.1#+(0.05*object_idx)
            object_position = [drop_x, drop_y, 0.15]
            object_orientation = [2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample()]
            vrep.simxSetObjectPosition(self.sim_client, object_handle, -1, object_position, vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(self.sim_client, object_handle, -1, object_orientation, vrep.simx_opmode_blocking)
            time.sleep(2)
            object_idx=object_idx+1
    def close_gripper(self, async=False):

        if self.is_sim:
            gripper_motor_velocity = -1.5
            gripper_motor_force = 1000
            sim_ret, gripper_handle_1 = vrep.simxGetObjectHandle(self.sim_client, 'panda_LeftFinger_joint', vrep.simx_opmode_blocking)
            sim_ret, gripper_handle_2 = vrep.simxGetObjectHandle(self.sim_client, 'panda_RightFinger_joint', vrep.simx_opmode_blocking)
            sim_ret, gripper_joint_position_1 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_1, vrep.simx_opmode_blocking)
            sim_ret, gripper_joint_position_2 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_2, vrep.simx_opmode_blocking)
            vrep.simxSetJointForce(self.sim_client, gripper_handle_1, gripper_motor_force, vrep.simx_opmode_blocking)
            vrep.simxSetJointForce(self.sim_client, gripper_handle_2, gripper_motor_force, vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(self.sim_client, gripper_handle_1, gripper_motor_velocity, vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(self.sim_client, gripper_handle_2, gripper_motor_velocity, vrep.simx_opmode_blocking)
            gripper_fully_closed = False
            gripper_pos=abs(gripper_joint_position_2-gripper_joint_position_1)
            gripper_joint_position=gripper_pos
            while (gripper_joint_position)>1.43 : # Block until gripper is fully closed
                sim_ret, new_gripper_joint_position_1 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_1, vrep.simx_opmode_blocking)
                sim_ret, new_gripper_joint_position_2 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_2, vrep.simx_opmode_blocking)
                # print(gripper_joint_position)
                new_gripper_joint_position=abs(new_gripper_joint_position_2-new_gripper_joint_position_1)
                if new_gripper_joint_position >= gripper_joint_position:
                    return gripper_fully_closed
                gripper_joint_position = new_gripper_joint_position
            gripper_fully_closed = True
        print( gripper_fully_closed)
        return gripper_fully_closed


    def open_gripper(self, async=False):

        if self.is_sim:
            #print('gripper opened')
            gripper_motor_velocity = 0.5
            gripper_motor_force = 5
            sim_ret, gripper_handle_1 = vrep.simxGetObjectHandle(self.sim_client, 'panda_LeftFinger_joint', vrep.simx_opmode_blocking)
            sim_ret, gripper_handle_2 = vrep.simxGetObjectHandle(self.sim_client, 'panda_RightFinger_joint', vrep.simx_opmode_blocking)
            sim_ret, gripper_joint_position_1 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_1, vrep.simx_opmode_blocking)
            sim_ret, gripper_joint_position_2 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_2, vrep.simx_opmode_blocking)
            vrep.simxSetJointForce(self.sim_client, gripper_handle_1, gripper_motor_force, vrep.simx_opmode_blocking)
            vrep.simxSetJointForce(self.sim_client, gripper_handle_2, gripper_motor_force, vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(self.sim_client, gripper_handle_1, gripper_motor_velocity, vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(self.sim_client, gripper_handle_2, gripper_motor_velocity, vrep.simx_opmode_blocking)
            while (gripper_joint_position_1-gripper_joint_position_2) < 0: # Block until gripper is fully open
                sim_ret, gripper_joint_position_1 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_1, vrep.simx_opmode_blocking)
                sim_ret, gripper_joint_position_2 = vrep.simxGetJointPosition(self.sim_client, gripper_handle_2, vrep.simx_opmode_blocking)      
    def push_to(self,pose, move_step_size=0.01, single_step=False):
    
        # Get current position and orientation of target
        sim_ret, target_position = vrep.simxGetObjectPosition(self.sim_client, self.target_handle,-1,vrep.simx_opmode_blocking)
        sim_ret, target_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.target_handle, -1, vrep.simx_opmode_blocking)

        # Calculate the movement increments
        move_direction = pose[:3,-1] - target_position
        move_magnitude = np.linalg.norm(move_direction)
        move_step = move_step_size * move_direction / move_magnitude
        num_move_steps = int(np.ceil(move_magnitude / move_step_size))

        # Calculate the rotation increments
        rotation = np.asarray(transformations.euler_from_matrix(pose))
        rotation_step = rotation - target_orientation
        rotation_step[rotation >= 0] = 0.1
        rotation_step[rotation < 0] = -0.1
        num_rotation_steps = np.ceil((rotation - target_orientation) / rotation_step).astype(np.int)

        # Move and rotate to the target pose
        if not single_step:
          for i in range(max(num_move_steps, np.max(num_rotation_steps))):
            pos = target_position + move_step*min(i, num_move_steps)
            rot = [np.pi/2,target_orientation[1]+rotation_step[1]*min(i, num_rotation_steps[1]),np.pi/2]
            vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,pos,vrep.simx_opmode_blocking) 
            #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, rot, vrep.simx_opmode_blocking) 

        vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,pose[:3,-1],vrep.simx_opmode_blocking) 
        #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, (np.pi/2,rotation[1],np.pi/2), vrep.simx_opmode_blocking)   


    def move_to(self, tool_position, tool_orientation):

        if self.is_sim:
            #print('moving grippper to target')
            # sim_ret, target_handle = vrep.simxGetObjectHandle(self.sim_client,'panda_targetTip',vrep.simx_opmode_blocking)
            sim_ret, target_position = vrep.simxGetObjectPosition(self.sim_client, self.target_handle,-1,vrep.simx_opmode_blocking)

            move_direction = np.asarray([tool_position[0] - target_position[0], tool_position[1] - target_position[1], tool_position[2] - target_position[2]])
            move_magnitude = np.linalg.norm(move_direction)
            move_step = 0.01*move_direction/move_magnitude
            num_move_steps = int(np.floor(move_magnitude/0.02))

            for step_iter in range(num_move_steps):
                vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(target_position[0] + move_step[0], target_position[1] + move_step[1], target_position[2] + move_step[2]),vrep.simx_opmode_blocking)
                sim_ret, target_position = vrep.simxGetObjectPosition(self.sim_client,self.target_handle,-1,vrep.simx_opmode_blocking)
            vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(tool_position[0],tool_position[1],tool_position[2]),vrep.simx_opmode_blocking)

    def get_camera_data(self):

        if self.is_sim:

            # Get color image from simulation
            sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.cam_handle, 0, vrep.simx_opmode_blocking)
            color_img = np.asarray(raw_image)
            color_img.shape = (resolution[1], resolution[0], 3)
            color_img = color_img.astype(np.float)/255
            color_img[color_img < 0] += 1
            color_img *= 255
            color_img = np.fliplr(color_img)
            color_img = color_img.astype(np.uint8)
            
            # Get depth image from simulation
            sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.cam_handle, vrep.simx_opmode_blocking)
            depth_img = np.asarray(depth_buffer)
            depth_img.shape = (resolution[1], resolution[0])
            depth_img = np.fliplr(depth_img)
            zNear = 0.01
            zFar = 10
            depth_img = depth_img * (zFar - zNear) + zNear

        return color_img, depth_img
   


   

# Primitives ----------------------------------------------------------

    def grasp(self, position, heightmap_rotation_angle, workspace_limits,set_position,orientation,set_flag=0):
        print('Executing: grasp at (%0.16f, %0.16f, %0.16f)' % (position[0], position[1], position[2]))

        if self.is_sim:

            # Compute tool orientation from heightmap rotation angle
            tool_rotation_angle = (heightmap_rotation_angle % np.pi) - np.pi/2

            # Avoid collision with floor
            position = np.asarray(position).copy()
            #position[2] = max(position[2]-0.04, workspace_limits[2][0] + 0.002)

            # Move gripper to location above grasp target
            grasp_location_margin = 0.09
            # sim_ret, target_handle = vrep.simxGetObjectHandle(self.sim_client,'',vrep.simx_opmode_blocking)
            location_above_grasp_target = (position[0], position[1], position[2] + grasp_location_margin)
            
            # Compute gripper position and linear movement increments
            tool_position = location_above_grasp_target
            sim_ret, target_position = vrep.simxGetObjectPosition(self.sim_client, self.target_handle,-1,vrep.simx_opmode_blocking)
            move_direction = np.asarray([tool_position[0] - target_position[0], tool_position[1] - target_position[1], tool_position[2] - target_position[2]])
            move_magnitude = np.linalg.norm(move_direction)
            move_step = 0.01*move_direction/move_magnitude
            num_move_steps = int(np.floor(move_direction[0]/move_step[0]))

            # Compute gripper orientation and rotation increments
            sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.target_handle, -1, vrep.simx_opmode_blocking)
            rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
            num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[1])/rotation_step))
             

            # Simultaneously move and rotate gripper
            for step_iter in range(max(num_move_steps, num_rotation_steps)):
                #print(step_iter)
                vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(target_position[0] + move_step[0]*min(step_iter,num_move_steps), target_position[1] + move_step[1]*min(step_iter,num_move_steps), target_position[2] + move_step[2]*min(step_iter,num_move_steps)),vrep.simx_opmode_blocking)
                #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, (np.pi/2, gripper_orientation[1] + rotation_step*0.05, np.pi/2), vrep.simx_opmode_blocking)
            vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(tool_position[0],tool_position[1],tool_position[2]),vrep.simx_opmode_blocking)
            #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, (np.pi/2,gripper_orientation[1] , np.pi/2), vrep.simx_opmode_blocking)

            # Ensure gripper is open
            self.open_gripper()
            
           
            # Approach grasp target
            self.move_to(position, None)

            # Close gripper to grasp target
            gripper_full_closed = self.close_gripper()
            grasp_success = not gripper_full_closed
            time.sleep(1.0)
            # Move gripper to location above grasp target
            self.move_to(location_above_grasp_target, None)

            # Check if grasp is successful
            #gripper_full_closed = self.close_gripper()
            

            # Move the grasped object elsewhere
            if set_flag==1:
                  pose=self.object_pose(set_position,orientation)
                  self.push_to(pose, single_step=False)
                  print(" Moving the grasped object ")                 
                  self.open_gripper()
                  set_position=(set_position[0],set_position[1], set_position[2] + 0.09)
                  self.move_to(set_position, None)
                  #grasp_success = not gripper_full_closed
                #vrep.simxSetObjectPosition(self.sim_client,grasped_object_handle,-1,(-0.5, 0.5 + 0.05*float(grasped_object_ind), 0.1),vrep.simx_opmode_blocking)

        return grasp_success

    def push(self, position, heightmap_rotation_angle, workspace_limits):
        position = np.asarray(position)
        print('Executing: push at (%f, %f, %f)' % (position[0], position[1], position[2]))

        if self.is_sim:

            # Compute tool orientation from heightmap rotation angle
            tool_rotation_angle = (heightmap_rotation_angle % np.pi) - np.pi/2

            # Adjust pushing point to be on tip of finger
            position[2] = position[2] + 0.026

            # Compute pushing direction
            direction_val=['right','left','top','down']
            direc=random.choice(direction_val)
            print('pushing towrds',direc)
            if(direc=='right'):
              position[0]=position[0]-0.09            
              push_orientation=[1.0,0.0]
            elif(direc=='left'):
              position[0]=position[0]+0.09
              push_orientation=[-1.0,0.0]
            elif(direc=='top'):
              position[1]=position[1]+0.09
              push_orientation=[0.0,-1.0]
            elif(direc=='down'):
              position[1]=position[1]-0.09
              push_orientation=[0.0,1.0]
            push_direction = np.asarray([push_orientation[0]*np.cos(heightmap_rotation_angle) - push_orientation[1]*np.sin(heightmap_rotation_angle), push_orientation[0]*np.sin(heightmap_rotation_angle) + push_orientation[1]*np.cos(heightmap_rotation_angle)])

            # Move gripper to location above pushing point
            pushing_point_margin = 0.1
            location_above_pushing_point = (position[0], position[1], position[2] + pushing_point_margin)
            
            # Compute gripper position and linear movement increments
            tool_position = location_above_pushing_point
            sim_ret, target_position = vrep.simxGetObjectPosition(self.sim_client, self.target_handle,-1,vrep.simx_opmode_blocking)
            move_direction = np.asarray([tool_position[0] - target_position[0], tool_position[1] - target_position[1], tool_position[2] - target_position[2]])
            move_magnitude = np.linalg.norm(move_direction)
            move_step = 0.05*move_direction/move_magnitude
            num_move_steps = int(np.floor(move_direction[0]/move_step[0]))

            # Compute gripper orientation and rotation increments
            sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.target_handle, -1, vrep.simx_opmode_blocking)
            rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
            num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[1])/rotation_step))

            # Simultaneously move and rotate gripper
            for step_iter in range(max(num_move_steps, num_rotation_steps)):
                vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(target_position[0] + move_step[0]*min(step_iter,num_move_steps), target_position[1] + move_step[1]*min(step_iter,num_move_steps), target_position[2] + move_step[2]*min(step_iter,num_move_steps)),vrep.simx_opmode_blocking)
                #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, (np.pi/2, gripper_orientation[1] + rotation_step*min(step_iter,num_rotation_steps), np.pi/2), vrep.simx_opmode_blocking)
            vrep.simxSetObjectPosition(self.sim_client,self.target_handle,-1,(tool_position[0],tool_position[1],tool_position[2]),vrep.simx_opmode_blocking)
            #vrep.simxSetObjectOrientation(self.sim_client, self.target_handle, -1, (np.pi/2, tool_rotation_angle, np.pi/2), vrep.simx_opmode_blocking)

            # Ensure gripper is closed
            self.close_gripper()

            # Approach pushing point
            self.move_to(position, None)

            # Compute target location (push to the right)
            push_length = 0.1
            target_x = min(position[0] + push_direction[0]*push_length, workspace_limits[0][1])
            target_y = min(position[1] + push_direction[1]*push_length, workspace_limits[1][1])
            push_length = np.sqrt(np.power(target_x-position[0],2)+np.power(target_y-position[1],2))

            # Move in pushing direction towards target location
            self.move_to([target_x, target_y, position[2]], None)

            # Move gripper to location above grasp target
            self.move_to([target_x, target_y, location_above_pushing_point[2]], None)

            push_success = True
        return push_success



def main(args):


        # --------------- Setup options ---------------
        is_sim = True
        obj_mesh_dir = '/home/uthira/Panda/objects/tower' if is_sim else None # Directory containing 3D mesh files (.obj) of objects to be added to simulation
        num_obj = 4 if is_sim else None # Number of objects to add to simulation
        if is_sim:
            workspace_limits = np.asarray([[0.724, 0.276], [-0.224, 0.224], [-0.0001, 0.4]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
        else:
            workspace_limits = np.asarray([[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
        heightmap_resolution = 0.002 # Meters per pixel of heightmap
        random_seed = 1234
        force_cpu = True   
        is_testing = True
        max_test_trials = args.max_test_trials # Maximum number of test runs per case/scenario
        test_preset_cases = False
        #head, tail = os.path.split('/home/uthira/visual-pushing-grasping/simulation/testcases/test-10-obj-01.txt')
        test_preset_file = args.test_preset_file if test_preset_cases else None
        # Set random seed
        np.random.seed(random_seed)
        # Initialize pick-and-place system (camera and robot)
        robot = Robot(is_sim, obj_mesh_dir, num_obj, workspace_limits,test_preset_file,heightmap_resolution,is_testing=True, test_preset_cases=False )   

    
    
    
    
        print('Testing' )
        
        # Make sure simulation is still stable (if not, reset simulation)
        if is_sim: # robot.check_sim()

        # Get latest RGB-D image
        #color_img, depth_img = robot.get_camera_data()
        #depth_img = depth_img * robot.cam_depth_scale # Apply depth scale from calibration

        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        #color_heightmap, depth_heightmap = utils.get_heightmap(color_img, depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
        #valid_depth_heightmap = depth_heightmap.copy()
        #valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

       
        # Reset simulation or pause real-world training if table is empty
        #stuff_count = np.zeros(valid_depth_heightmap.shape)
        #stuff_count[valid_depth_heightmap > 0.02] = 1
        #empty_threshold = 300
        #if is_sim and is_testing:
        #empty_threshold =4  
        #if np.sum(stuff_count) < empty_threshold: 
         #   if is_sim:
          #      print('Not enough objects in view (value: %d)! Repositioning objects.' % (np.sum(stuff_count)))
           #     robot.restart_sim()
            #    robot.add_objects()
         nonlocal_variables = {'best_pix_ind' : None} 
         print("**** Buidling and Breaking a tower of Blocks !!****")
         for i in range(1,10):
           print(".....")
           print("Iteration :"+str(i))      
           robot.init_tower()        
           myarray = np.asarray(robot.random_obj())         
           nonlocal_variables['best_pix_ind']=tuple(myarray)
           best_pix_x = nonlocal_variables['best_pix_ind'][0]
           best_pix_y = nonlocal_variables['best_pix_ind'][1]
           best_pix_z = nonlocal_variables['best_pix_ind'][2]         
           zz=int(best_pix_z)              
           best_rotation_angle = np.deg2rad(zz*(360.0/16))         
           primitive_position = [best_pix_x , best_pix_y ,best_pix_z]           
           #p=robot.push(primitive_position, best_rotation_angle, workspace_limits)
           #print("Push Sucessful :"+str(p))         
           time.sleep(2.0)
           workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
           robot.reposition_objects(workspace_limits)
     
        

               
        

if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser(description='Train robotic agents to learn how to plan complementary pushing and grasping actions for manipulation with deep reinforcement learning in PyTorch.')

   #parser.add_argument('--heightmap_resolution', dest='heightmap_resolution', type=float, action='store', default=0.002, help='meters per pixel of heightmap')
    
    #parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,                                    help='force code to run in CPU mode')

    #parser.add_argument('--is_testing', dest='is_testing', action='store_true', default=True)
    parser.add_argument('--max_test_trials', dest='max_test_trials', type=int, action='store', default=30,                help='maximum number of test runs per case/scenario')
    #parser.add_argument('--test_preset_cases', dest='test_preset_cases', action='store_true', default=False)
    parser.add_argument('--test_preset_file', dest='test_preset_file', action='store', default='test-10-obj-03.txt')
    
   

    #parser.add_argument('--load_snapshot', dest='load_snapshot', action='store_true', default=False,                      help='load pre-trained snapshot of model?')
    #parser.add_argument('--snapshot_file', dest='snapshot_file', action='store')
    #parser.add_argument('--continue_logging', dest='continue_logging', action='store_true', default=False,                help='continue logging from previous session?')
    #parser.add_argument('--logging_directory', dest='logging_directory', action='store')


    args = parser.parse_args()
    main(args)
