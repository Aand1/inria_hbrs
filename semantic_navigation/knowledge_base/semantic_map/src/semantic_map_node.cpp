// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <semantic_map/semantic_map.h>


// Standard C++ entry point
int main(int argc, char** argv) 
{
  // Initialize ROS
  ros::init(argc, argv, "semantic_map_node");
  ros::NodeHandle nh;
  
  semantic_map::SemanticMap* semantic_map = new semantic_map::SemanticMap(nh);
  //semantic_map->createMap();
  //semantic_map->updateMap();
  
  // Wait for SIGINT/Ctrl-C
  //ros::spin();

  return 0;
}
