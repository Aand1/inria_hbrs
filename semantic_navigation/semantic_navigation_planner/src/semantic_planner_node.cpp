// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <semantic_navigation_planner/semantic_planner.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h> 



// Standard C++ entry point
int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "semantic_planner_node");
    ros::NodeHandle nh;

    tf::TransformListener tf(ros::Duration(10));
  
    ROS_INFO("semantic_planner_node is now running");

    semantic_navigation_planner::SemanticPlanner* semantic_planner = new semantic_navigation_planner::SemanticPlanner( tf );

    
    ros::spin();

    return 0;
}
