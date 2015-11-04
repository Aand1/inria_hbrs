#ifndef CLIENT_GOAL_H
#define CLIENT_GOAL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <semantic_knowledgebase/MoveObjectGoal.h>
#include <cstdlib>

	class ClientGoal{
	public:	
		ClientGoal(const ros::NodeHandle &nh);
		~ClientGoal();

		semantic_knowledgebase::Goal& call_service(std_msgs::String object_category_);

	private:	
		ros::ServiceClient client_;
		
        	ros::NodeHandle nh_;
		std_msgs::String object_category_; 
		semantic_knowledgebase::Goal goal_;
		


	};


#endif
