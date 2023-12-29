#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties, SetModelState

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:
	
    object_names = []
    invariable_object_names = {'ground_plane','workcell-assembly-v2-kinnect','arm'}

    _blockListDict = {
        'block_a': Block('mobile_base', 'wheel_left_link'),
        'block_b': Block('brick_box_3x1x3', 'world'),

    }


    def pause_gazebo_physics(self):
		pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
		pause()
		print('gazebo paused')
	
    def unpause_gazebo_physics(self):
		unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
		unpause()
		print('gazebo unpaused')
			
    def get_object_names(self):
        get_world_prop=rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        try:
            world_prop = get_world_prop()
            print('object names:', world_prop.model_names)
            object_names = world_prop.model_names
        except rospy.ServiceException as e:
            rospy.loginfo("Get World Properties service call failed:  {0}".format(e))
		
		
    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                print '\n'
                print 'Status.success = ', resp_coordinates.success
                print(blockName)
                print("Cube " + str(block._name))
                print("Valeur de X : " + str(resp_coordinates.pose.position.x))
                print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    tuto = Tutorial()
    tuto.pause_gazebo_physics()
    tuto.get_object_names()
	#for model in resp1.model_names:
    tuto.show_gazebo_models()
    tuto.unpause_gazebo_physics()
    

#rosservice call /gazebo/get_world_properties
