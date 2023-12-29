#!/usr/bin/env python

""" cv_bridge_demo.py - Version 0.1 2011-05-29
    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

#import roslib; roslib.load_manifest('rbx1_vision')
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Depth_subs():
    
    depth_image_normalized = None
    
    def __init__(self):
        self.node_name = "depth_subscriber"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        #self.cv_window_name = self.node_name
        #cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        #cv2.moveWindow("image", 25, 75)
        
        # And one for the depth image
        #cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
        #cv2.moveWindow("depth", 25, 350)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        #self.image_sub = rospy.Subscriber("/kinect_camera/depth/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/kinect_camera/depth/depth_image_raw", Image, self.depth_callback)
        
        rospy.loginfo("Waiting for image topics...")



    def depth_callback(self, ros_image):
        rate = rospy.Rate(35) # 10hz
        cnt = -1
        #print 'depth_callback'
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            #depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
            #depth_array = np.array(depth_image, dtype=np.float32)
            self.depth_image_normalized = cv2.normalize(depth_image, None , 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            
            #print(depth_image_normalized.dtype)
            #print(depth_image.astype("uint8").dtype)
            #depth_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            
            #depth_image.convertTo(img2, CV_8UC1);
            #cv2.imshow("depth", self.depth_image_normalized)
            #self.keystroke = cv2.waitKey(500)
        except CvBridgeError, e:
            print e
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        #depth_array = np.array(depth_image, dtype=np.uint8)
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # Process the depth image
        #depth_display_image = self.process_depth_image(depth_array)
    
        # Display the result
        #cv2.imshow("Depth Image", depth_display_image)
        
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   

    
def main(args):       
    try:
        cnt = 1
        #rate = rospy.Rate(10) # 10hz
        obj = Depth_subs()
        #rospy.spin()
        while not rospy.is_shutdown():
            try:
                cv2.imshow("depth", obj.depth_image_normalized)
                cv2.waitKey(500)
            except:
				continue;
        #rospy.spin()
        print "depth callback."
    except KeyboardInterrupt:
        print "Shutting down vision node."
        DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
