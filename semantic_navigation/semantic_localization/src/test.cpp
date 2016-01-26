// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <semantic_map/query_regions.h>

// Standard C++ entry point
int main(int argc, char** argv) 
{
  // Initialize ROS
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  
  ROS_INFO("test_node is now running");

  // Call query_regions library function
  query_semantic_map::QueryRegions* query_regions = new query_semantic_map::QueryRegions(nh);
  semantic_map::RegionList region_list = query_regions->query();
  
  //The response received by teh Server for monitoring puropse
  //ROS_INFO_STREAM(region_list);


  // Wait for SIGINT/Ctrl-C
  ros::spin();
  return 0;
}