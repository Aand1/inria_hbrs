
#include <query_goal/client_goal.h>



ClientGoal::ClientGoal(const ros::NodeHandle &nh):nh_(nh) {
	client_ = nh_.serviceClient<semantic_knowledgebase::MoveObjectGoal>("move_object_get_goal");


}

ClientGoal::~ClientGoal() {}


semantic_knowledgebase::Goal& ClientGoal::call_service(std_msgs::String object_category_)
{
	if(client_.exists()) 
        {
		ROS_INFO("server found.....");
		semantic_knowledgebase::MoveObjectGoal goal_info;
		goal_info.request.object_category = object_category_.data;

		if(client_.call(goal_info)) {
		    goal_ = goal_info.response.goal;
                    ROS_INFO_STREAM("Response recieved");

                } else {
                    ROS_INFO("Response failed.....");
                }
                
         
                
        } 

	else 
	{
                ROS_INFO("server is not running.....");
        }

	return goal_;

}
