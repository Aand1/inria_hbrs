// Include the ROS C++ APIs
#include <ros/ros.h>

// Include the declaration of our library function
#include <semantic_navigation_planner/compute_best_path.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h> 



// Standard C++ entry point
int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "compute_best_path_node");
    ros::NodeHandle nh;

    tf::TransformListener tf(ros::Duration(10));
  
    ROS_INFO("compute_best_path_node is now running");

    semantic_navigation_planner::ComputeBestPath* best_path = new semantic_navigation_planner::ComputeBestPath( tf, nh );

    //best_path->run();
    //best_path->test();

    geometry_msgs::PoseStamped robot_pose, robot_goal;
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = "map";
    robot_pose.pose.position.x = 3.5;
    robot_pose.pose.position.y = -2.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 0.0;

    robot_goal.header.frame_id = "/map";
    robot_goal.pose.position.x = -2.5;
    robot_goal.pose.position.y = -1.80;

    
    best_path->getObjectPlanCost(robot_pose, robot_goal);
  
  /* ros::Publisher best_path_trigger_pub = nh.advertise<std_msgs::Bool>("best_path/trigger", 1000, true);
 
    std_msgs::Bool trigger;
    trigger.data = 1;
    
    ROS_INFO_STREAM("Sending trigger to best_path");
    best_path_trigger_pub.publish(trigger);*/
  
    // Wait for SIGINT/Ctrl-C
    ros::spin();

    return 0;
}
