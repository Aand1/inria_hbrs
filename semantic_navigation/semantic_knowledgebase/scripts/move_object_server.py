#!/usr/bin/env python

import rospy
import yaml
import copy

from semantic_knowledgebase.srv import *
from rospy_message_converter import message_converter
from semantic_knowledgebase.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

class ServerForMoveObjectGoal():
    def __init__(self, data):
	self.server = rospy.Service('move_object_get_goal', MoveObjectGoal, self.service_callback_for_get_goal)
    	self.data = data

    def service_callback_for_get_goal(self, request):
        #rospy.loginfo(request.object_category_request)
 
        goal = Goal()
        for t in self.data:
          goal.instance.name = t['instance']
          goal.goal = message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',t['goal'])

          rospy.loginfo(goal)

	return MoveObjectGoalResponse(goal)

def initlize_node():
    '''
    Initilize node and spin which simply keeps python 
    from exiting until this node is stopped
    '''
    rospy.init_node('move_object_goal_node', anonymous=False)
    #rospy.loginfo("object_information_all_node is now running")


    filename = rospy.get_param('~filename')
    with open(filename) as f:
         yaml_data = yaml.load(f)

    server_class = ServerForMoveObjectGoal(yaml_data)
    rospy.spin()

if __name__ == '__main__':
    initlize_node()
