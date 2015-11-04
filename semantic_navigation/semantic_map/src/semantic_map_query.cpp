
#include <semantic_map/semantic_map_query.h>

using namespace semantic_map;

SemanticMapQuery::SemanticMapQuery(const ros::NodeHandle &nh):nh_(nh) 
{
	semantic_map_query = nh_.serviceClient<semantic_map::SemanticMapQueryResponse>("semantic_map_query_response"); 
}

SemanticMapQuery::~SemanticMapQuery() 
{

}


semantic_map::SemanticMapMessage& SemanticMapQuery::query()
{

	if(semantic_map_query.exists()) 
    {
    	ROS_INFO_STREAM("Query send to semantic map");
        semantic_map::SemanticMapQueryResponse query;
    	if (semantic_map_query.call(query))
    	{
    		ROS_INFO_STREAM("Response recieved");

    		semantic_map_ = query.response.semantic_map_message;
    	}

    	else
    	{
    		ROS_INFO_STREAM("Response FAILED");
    	}


	}

	return semantic_map_;

}

