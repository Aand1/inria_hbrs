#!/usr/bin/env python

import rospy

from semantic_knowledgebase.srv import *
from semantic_knowledgebase.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *



def initlize_node():
    '''
    Initilize node and spin which simply keeps python 
    from exiting until this node is stopped
    '''
    rospy.init_node('move_object_goal_node_client_test', anonymous=False)
    rospy.loginfo("move_object_goal_node_client_test is now running")

    rospy.wait_for_service('move_object_get_goal')

    try:
        move_object_get_goal = rospy.ServiceProxy('move_object_get_goal', MoveObjectGoal)
        req = MoveObjectGoalRequest()
        req.object_category = ""
        resp = move_object_get_goal(req)
        rospy.loginfo(resp.goal.goal)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    
    rospy.spin()

if __name__ == '__main__':
    initlize_node()
