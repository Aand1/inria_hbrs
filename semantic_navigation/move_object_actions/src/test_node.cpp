#include <ros/ros.h>
#include <semantic_planner_global/semantic_planner_global.h>
#include <semantic_map/semantic_map.h>
#include <geometry_msgs/PoseStamped.h>  
#include <tf/transform_listener.h> 
#include <sem_nav_msgs/MoveObjectGoals.h>

 
#include <move_object_actions/PushAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<move_object_actions::PushAction> PushActionClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_node");	

	ros::NodeHandle nh_;
	tf::TransformListener tf_;

	semantic_planner::SemanticPlannerGlobal* planner_; 

	planner_ = new semantic_planner::SemanticPlannerGlobal( &tf_ );

	semantic_map::SemanticMap* semantic_map_query_;
	semantic_map_query_ = new semantic_map::SemanticMap(nh_);

	semantic_map::Object object;
	semantic_map_query_->getObject(object);

	sem_nav_msgs::MoveObjectGoals object_goals;
	object_goals.approach_object.header.frame_id = "/map";
    object_goals.approach_object.pose.position.x = 2.5;
    object_goals.approach_object.pose.position.y = -1.0;

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = "map";
    robot_pose.pose.position.x = 3.5;
    robot_pose.pose.position.y = -2.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 0.0;

	planner_->makePlanObjectApproach(robot_pose, object_goals.move_object, plan, object);

	//PushActionClient client("push_object", true);

	//client.waitForServer();

	//move_object_actions::PushGoal goal;

	ros::spin();
	return 0;

}
