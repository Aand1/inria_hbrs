#!/usr/bin/env python

import rospy
import yaml
import copy

from semantic_map_ros import SemanticMapROS
from semantic_map.srv import *

from rospy_message_converter import message_converter
from yocs_msgs.msg import *
from semantic_map.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

'''
This class waits for a query about information about different entities(objects and regions) of the environment stored in the semantic map. 
	
The response of the class is a EntityList message which contains different form of information about entitiess existing in the robot environment.
'''

class SemanticMapResponse(SemanticMapROS):
	def __init__(self):
		regions_filename = rospy.get_param('~regions_filename')
		with open(regions_filename) as f1:
			self.regions_data = yaml.load(f1)

		objects_filename = rospy.get_param('~objects_filename')
		with open(objects_filename) as f2:
			self.objects_data = yaml.load(f2)

		self.server = rospy.Service('semantic_map_query_response', SemanticMapQueryResponse, self.service_callback)	


	def service_callback(self, request):
		#rospy.loginfo("Semantic map information is available") 
		semantic_map = SemanticMapMessage()

		objects = Object()
		

		for t1 in self.regions_data:
			regions = Region()
			regions.instance.name = t1['instance']
			regions.geometry.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t1['geometry']['pose'])
			regions.geometry.bounding_box = message_converter.convert_dictionary_to_ros_message('semantic_knowledgebase/BoundingBox',t1['geometry']['bounding_box'])
			regions.semantics.category = t1['semantics']['category']
			regions.semantics.sub_category = t1['semantics']['sub_category']

			semantic_map.regions.append(regions)

		for t2 in self.objects_data:
			objects = Object()
			objects.instance.name = t2['instance']
			objects.geometry.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t2['geometry']['pose'])
			objects.geometry.bounding_box = message_converter.convert_dictionary_to_ros_message('semantic_knowledgebase/BoundingBox',t2['geometry']['bounding_box'])
			objects.semantics.category = t2['semantics']['category']
			objects.semantics.sub_category = t2['semantics']['sub_category']

			semantic_map.objects.append(objects)
            
			
		return SemanticMapQueryResponseResponse(semantic_map)

	


def initialize_node():
	'''
	Initialize the RegionsList class and spin to keep python from exiting untill this node is stopped.
	'''

	rospy.init_node('semantic_map_query_response', anonymous=False)
	#rospy.loginfo("regions_node is now running")

        semantic_map_response = SemanticMapResponse()

	rospy.spin()

if __name__ == '__main__':
	initialize_node()
