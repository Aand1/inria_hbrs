// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <semantic_navigation_planner/constraints.h>
#include <std_msgs/Int32.h>




// Standard C++ entry point
int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "constraints_node");
  
    ROS_INFO("constraints_node is now running");

    semantic_navigation_planner::Constraints* constraints_ = new semantic_navigation_planner::Constraints();

    std_msgs::Int32 input;
    input.data = 1;

    constraints_->computeConstraints(input);
  
    // Wait for SIGINT/Ctrl-C
    ros::spin();

    return 0;
}
