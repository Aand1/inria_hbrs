#ifndef _SEMANTIC_MAP_QUERY_H
#define _SEMANTIC_MAP_QUERY_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <semantic_map/SemanticMapQueryResponse.h>
#include <cstdlib>

namespace semantic_map
{

	class SemanticMapQuery
	{
	public:	
		SemanticMapQuery(const ros::NodeHandle &nh);
		~SemanticMapQuery();

		semantic_map::SemanticMapMessage& query();

	private:	
		ros::ServiceClient semantic_map_query;
		
        ros::NodeHandle nh_;
		semantic_map::SemanticMapMessage semantic_map_;

	};
}

#endif
