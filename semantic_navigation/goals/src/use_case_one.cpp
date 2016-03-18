//for testing object pushing
#include <ros/ros.h>
#include <sem_nav_msgs/SemanticPlannerAction.h>
#include <actionlib/client/simple_action_client.h>



typedef actionlib::SimpleActionClient<sem_nav_msgs::SemanticPlannerAction> SemanticPlannerActionClient;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "use_case_one_goal_node");

    SemanticPlannerActionClient ac("semantic_planner", true);  

    bool reached_object;
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the semantic planner action server to come up");
    }

    //////////////Manually create goal in behalf of task planner/////////////
    sem_nav_msgs::SemanticPlannerGoal goal;

    goal.input.geometric_goal;
    goal.input.geometric_goal.header.frame_id = "/map";
    goal.input.geometric_goal.header.stamp = ros::Time::now();
    goal.input.geometric_goal.pose.position.x = -1.0;
    goal.input.geometric_goal.pose.position.y = 0.7;
    goal.input.geometric_goal.pose.orientation.w = 1.0;

    goal.object_instance.name = "box";

    //goal.input.purpose.data = "emergency";
    //////////////////////////////////////////////////
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the semantic planner executed goal");        
    }
    else
        ROS_INFO("The semantic planner failed");
    

    
    return 0;

}