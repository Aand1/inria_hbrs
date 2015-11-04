// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <query_goal/client_goal.h>

// Standard C++ entry point
int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "client_goal_node");
  ros::NodeHandle nh;
  std_msgs::String object_instance;
  object_instance.data = "";

  ROS_INFO("client_goal_node is now running");

  // Call our library function
  ClientGoal* client_goal = new ClientGoal(nh);
  semantic_knowledgebase::Goal goal_info_ = client_goal->call_service(object_instance);
  ROS_INFO_STREAM(goal_info_);


  // Wait for SIGINT/Ctrl-C
  ros::spin();
  return 0;
}
