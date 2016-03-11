//To test whether the archi uses semantic info about regions
#include <ros/ros.h>
#include <sem_nav_msgs/SemanticPlannerAction.h>
#include <actionlib/client/simple_action_client.h>



typedef actionlib::SimpleActionClient<sem_nav_msgs::SemanticPlannerAction> SemanticPlannerActionClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "use_case_three_one_node");

    SemanticPlannerActionClient ac("semantic_planner", true);  

    bool reached_object;
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the semantic planner action server to come up");
    }

    //////////////Manually create goal in behalf of task planner/////////////
    sem_nav_msgs::SemanticPlannerGoal goal_one, goal_two, goal_three;

    /////////////Goal to reach doorway entrance/////////////
    goal_one.input.geometric_goal;
    goal_one.input.geometric_goal.header.frame_id = "/map";
    goal_one.input.geometric_goal.header.stamp = ros::Time::now();
    goal_one.input.geometric_goal.pose.position.x = -0.850;
    goal_one.input.geometric_goal.pose.position.y = 2.6;
    goal_one.input.geometric_goal.pose.orientation.w = 1.5;


    /////////////Goal to reach doorway exit/////////////////
    goal_two.input.geometric_goal;
    goal_two.input.geometric_goal.header.frame_id = "/map";
    goal_two.input.geometric_goal.header.stamp = ros::Time::now();
    goal_two.input.geometric_goal.pose.position.x = -0.850;
    goal_two.input.geometric_goal.pose.position.y = 5.5;
    goal_two.input.geometric_goal.pose.orientation.w = 1.5;


    /////////////Goal to reach corridor/////////////////
    goal_three.input.geometric_goal;
    goal_three.input.geometric_goal.header.frame_id = "/map";
    goal_three.input.geometric_goal.header.stamp = ros::Time::now();
    goal_three.input.geometric_goal.pose.position.x = 3.2;
    goal_three.input.geometric_goal.pose.position.y = 5.5;
    goal_three.input.geometric_goal.pose.orientation.w = 0.1;


    //////////////////////////////////////////////////
    ac.sendGoal(goal_one);

    ac.waitForResult();

    while ( !(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) )
    {

    }
    ac.sendGoal(goal_two);

    ac.waitForResult();

    while ( !(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) )
    {
        
    }
    ac.sendGoal(goal_three);

    ac.waitForResult();

    while ( !(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) )
    {
        
    }

    ROS_INFO("Hooray, the semantic planner executed goal");
    
    return 0;

}
