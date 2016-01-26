// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <perception/dummy_perception.h>


// Standard C++ entry point
int main(int argc, char** argv) 
{
  // Initialize ROS
  ros::init(argc, argv, "dummy_perception_node");
  ros::NodeHandle nh;
  
  ROS_INFO("dummy_perception_node is now running");

  // Call query_regions library function
  //query_semantic_map::QueryRegions* query_regions = new query_semantic_map::QueryRegions(nh);
  //semantic_map::RegionList region_list = query_regions->query();

  DummyPerception* dummy_perception = new DummyPerception(nh);
  dummy_perception->updateMap();
  //dummy_perception->queryGazebo();

//  while (ros::ok())
//  {
//    dummy_perception->queryGazebo();
//  }  
  // Wait for SIGINT/Ctrl-C
  ros::spin();

  return 0;
}
