#ifndef TEST_ACTION_H_
#define TEST_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sem_nav_msgs/PushAction.h>
#include <sem_nav_msgs/MoveObjectGoals.h>

#include <semantic_planner_global/semantic_planner_global.h> 
#include <semantic_map/semantic_map.h>
#include <costmap_2d/costmap_2d_ros.h>

//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<sem_nav_msgs::PushAction> PushActionServer;

class TestAction 
{
	public:
		TestAction(tf::TransformListener& tf, ros::NodeHandle &nh); 

        virtual ~TestAction();

        void executeCb(const sem_nav_msgs::PushGoalConstPtr& object_instance);

        bool executeCycle(const sem_nav_msgs::MoveObjectGoals& goals);

    private:

  	    

  	    void makePlan();
  	    void planThread();

  	    PushActionServer* as_;

    	ros::NodeHandle nh_;
		tf::TransformListener& tf_;

		semantic_planner::SemanticPlannerGlobal* planner_; 

		//Instance of SemanticMap class to query the semantic map
		semantic_map::SemanticMap* semantic_map_query_; 

		boost::thread* planner_thread_;
		bool runPlanner_;

		costmap_2d::Costmap2DROS* planner_costmap_ros_;

};

#endif    