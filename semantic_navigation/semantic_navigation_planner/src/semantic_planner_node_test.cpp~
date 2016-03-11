#include <ros/ros.h>
#include <sem_nav_msgs/PushAction.h>
#include <actionlib/client/simple_action_client.h>
#include <semantic_navigation_planner/compute_best_path.h>


typedef actionlib::SimpleActionClient<sem_nav_msgs::PushAction> PushActionClient;

int main(int argc, char** argv)
{
   ros::init(argc, argv, "semantic_planner_node_node");

   //tell the action client that we want to spin a thread by default
/*   PushActionClient ac("push_action", true);

   //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the push action server to come up");
    }

    sem_nav_msgs::PushGoal push_goal;
    push_goal.object_instance.name = "corrogated_box2";

    push_goal.reach_goal.header.frame_id = "map";
    push_goal.reach_goal.pose.position.x = -0.50;
    push_goal.reach_goal.pose.position.y = 2.0;
    push_goal.reach_goal.pose.orientation.w = 1.0;


    // Fill goal with values. For now hard coded.

    ROS_INFO("Sending trigger to PushAction");
    ac.sendGoal(push_goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Hooray, the base pushed the object");
     else
       ROS_INFO("The base failed to pushed the object");*/

    PushActionClient ac("push_action", true);  
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the push action server to come up");
    }

    //////////////////////////////////////////////////////////////////
    tf::TransformListener tf(ros::Duration(10));
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped start, goal;
    start.header.frame_id = "map";
    start.pose.position.x = 3.0;

    goal.header.frame_id = "map";
    goal.pose.position.x = -0.5;

    semantic_navigation_planner::ComputeBestPath* best_path = new semantic_navigation_planner::ComputeBestPath(tf, nh); 

    sem_nav_msgs::BestPath bp;
    bp = best_path->computeBestPath(start, goal);


    ros::Duration(3).sleep();


    if ( (bp.category.compare("Box") == 0) ) 
    {
        sem_nav_msgs::PushGoal push_goal;
        push_goal.object_instance.name = bp.instance;

        push_goal.reach_goal.header.frame_id = "map";
        push_goal.reach_goal.pose.position.x = goal.pose.position.x;
        push_goal.reach_goal.pose.position.y = 0.0;
        push_goal.reach_goal.pose.orientation.w = 0.0;

        ROS_INFO("Sending trigger to PushAction");
        ac.sendGoal(push_goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("Hooray, the base pushed the object");
        else
          ROS_INFO("The base failed to pushed the object");
    }
    
    return 0;

}
