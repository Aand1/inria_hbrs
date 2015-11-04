
#include <semantic_map/query_regions.h>

using namespace query_semantic_map;

QueryRegions::QueryRegions(const ros::NodeHandle &nh):nh_(nh) 
{
	query_regions = nh_.serviceClient<semantic_map::QueryRegionInstances>("query_region_instances");
}

QueryRegions::~QueryRegions() {}


semantic_map::RegionList& QueryRegions::query()
{
	if(query_regions.exists()) 
    {
    	semantic_map::QueryRegionInstances regions;
    	
    	if (query_regions.call(regions))
    	{
    		ROS_INFO_STREAM("Response recieved");

    		region_list = regions.response.region_list;
    	}

    	else
    	{
    		ROS_INFO_STREAM("Response FAILED");
    	}


	}

	return region_list;

}

