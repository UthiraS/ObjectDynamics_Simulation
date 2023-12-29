#!/usr/bin/python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy

import tf
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties, GetModelState, SetModelState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Wrench, Pose, Twist

def handle_gazebo_links(linkstates):
    ind =  linkstates.name.index('workcell-assembly-v2-kinnect::camera_link_optical')
    #print('ind=',ind)
    #print(linkstates.pose[ind].position)
    #print(linkstates.pose[ind].orientation)
    #kinect_pose = Pose()
    
    br = tf.TransformBroadcaster()
    
    br.sendTransform((linkstates.pose[ind].position.x,linkstates.pose[ind].position.y,linkstates.pose[ind].position.z),
                     (linkstates.pose[ind].orientation.x,linkstates.pose[ind].orientation.y,linkstates.pose[ind].orientation.z,linkstates.pose[ind].orientation.w),
                     rospy.Time.now(),
                     'camera_link',
                     'world')

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    link_sub = rospy.Subscriber('/gazebo/link_states',
                     LinkStates,
                     handle_gazebo_links)
    rospy.spin()
