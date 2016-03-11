#include <ros/ros.h>
#include <sem_nav_msgs/SemanticPlannerAction.h>
#include <actionlib/client/simple_action_client.h>



typedef actionlib::SimpleActionClient<sem_nav_msgs::SemanticPlannerAction> SemanticPlannerActionClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "use_case_two_goal_node");

    SemanticPlannerActionClient ac("semantic_planner", true);  

    bool reached_object;
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the semantic planner action server to come up");
    }

    //////////////Manually create goal in behalf of task planner/////////////
    sem_nav_msgs::SemanticPlannerGoal goal_one, goal_two;

    /////////////Goal to reach object for manipulation/////////////
    goal_one.input.geometric_goal;
    goal_one.input.geometric_goal.header.frame_id = "/map";
    goal_one.input.geometric_goal.header.stamp = ros::Time::now();
    goal_one.input.geometric_goal.pose.position.x = 0.0;
    goal_one.input.geometric_goal.pose.position.y = 2.0;
    goal_one.input.geometric_goal.pose.orientation.w = 1.0;

    goal_one.input.purpose.data = "reach";

    /////////////Goal to carry object/////////////////
    goal_two.input.geometric_goal;
    goal_two.input.geometric_goal.header.frame_id = "/map";
    goal_two.input.geometric_goal.header.stamp = ros::Time::now();
    goal_two.input.geometric_goal.pose.position.x = 0.0;
    goal_two.input.geometric_goal.pose.position.y = 3.0;
    goal_two.input.geometric_goal.pose.orientation.w = 1.0;

    goal_two.input.purpose.data = "carry";


    //////////////////////////////////////////////////
    ac.sendGoal(goal_one);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        //ROS_INFO("Hooray, the semantic planner executed goal");
        reached_object = true;
        
    }
    else
        ROS_INFO("The semantic planner failed");

    ros::Duration(2).sleep();

    if (reached_object == true)
    {
        ac.sendGoal(goal_two);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, the semantic planner executed goal");
        }
        else
            ROS_INFO("The semantic planner failed");

    }
    

    
    return 0;

}
