#ifndef _QUERY_REGIONS_H
#define _QUERY_REGIONS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <semantic_map/QueryRegionInstances.h>
#include <cstdlib>

namespace query_semantic_map{

	class QueryRegions{
	public:	
		QueryRegions(const ros::NodeHandle &nh);
		~QueryRegions();

		semantic_map::RegionList& query();

	private:	
		ros::ServiceClient query_regions;
		
        ros::NodeHandle nh_;
		semantic_map::RegionList region_list;

	};
}

#endif
