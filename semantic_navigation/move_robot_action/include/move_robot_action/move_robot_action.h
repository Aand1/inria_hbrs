/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg, Germany
 *                      Inria, France
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Hochschule Bonn-Rhein-Sieg, Germany and Inria, 
 *     France nor the names of its contributors may be used to 
 *     endorse or promote products derived from this software without 
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Niranjan Vilas Deshpande
 *         (niranjan.deshpande187@gmail.com)
 *
 * Supervised by: Sven Schneider (Hochschule Bonn-Rhein-Sieg)
 *                Prof. Dr. Paul G. Ploeger (Hochschule Bonn-Rhein-Sieg)
 *		          Dr. Anne Spalanzani (Inria)
 *********************************************************************/

#ifndef MOVE_ROBOT_ACTION_H_
#define MOVE_ROBOT_ACTION_H_ 


// ROS related
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> 
#include <sem_nav_msgs/MoveRobotAction.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <semantic_costmap/semantic_costmap_ros.h> 
#include <semantic_planner_global/semantic_planner_global.h>
#include <semantic_planner_local/semantic_planner_local.h>
#include <sem_nav_msgs/GetPlanObject.h> 
#include <sem_nav_msgs/SemanticPose.h> 
#include <sem_nav_msgs/Constraints.h> 

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <goal_monitor/goal_monitor.h>

#include <pluginlib/class_loader.h> 

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>  

namespace move_robot_action 
{

	//typedefs to help us out with the action server so that we don't hace to type so much
	typedef actionlib::SimpleActionServer<sem_nav_msgs::MoveRobotAction> MoveRobotActionServer;

	enum MoveRobotState 
	{
	    PLANNING,
	    CONTROLLING,
	    STOP
    };

    /*
     * @class MoveRobot
     * @brief A class that uses the actionlib::ActionServer interface to move the robot to a geometric goal location.
     */
	class MoveRobotAction 
    {
	public:
		/*
         * @brief  Constructor for the MoveRobot object
         * @param name The name of the class
         */
		MoveRobotAction(tf::TransformListener& tf);

		/*
         * @brief  Destructor - Cleans up
         */
		virtual ~MoveRobotAction();

		/*
          * @brief  Initialization function for the MoveRobot object
          * @param  tf A reference to a TransformListener
          * @param  planner_costmap A pointer to the global costmap to use for global planning
          */
		void initialize();

		/*
         * @brief  Performs a control cycle
         * @param goal A reference to the goal to pursue
         * @param global_plan A reference to the global plan being used
         * @return True if processing of the goal is done, false otherwise
         */
        bool executeCycle(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    protected:
		bool initialized_;

	private:

        bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

        void localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose);

		/*
         * @brief  Make a new global plan
         * @param  goal The goal to plan to
         * @param  plan Will be filled in with the plan made by the planner
         * @return  True if planning succeeds, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /* @brief Separate thread to activate makePlan function. This thread runs periodically to while the robot is achieving its goal to deal with dynamic obstacles. 
         */
        void planThread();

        /*
         * @brief To wake the planner at periodic intervals.
         */
        void wakePlanner(const ros::TimerEvent& event);

        void executeCb(const sem_nav_msgs::MoveRobotGoalConstPtr& move_robot_goal);

        bool isQuaternionValid(const geometry_msgs::Quaternion& q);
		
		geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

		void publishZeroVelocity();

		void resetState();

        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

        void constraintsCB(const sem_nav_msgs::Constraints::ConstPtr& constraints);
        void updateSemanticCostmapConstraints(const sem_nav_msgs::SemanticCostmapConstraints& semantic_costmap_constraints);
        void updateLocalPlannerConstraints(const sem_nav_msgs::LocalPlannerConstraints& lpc);

		tf::TransformListener& tf_;
		MoveRobotActionServer* as_;

		/**
         * @brief To store copies of global and local costmaps
         */
        semantic_costmap::SemanticCostmapROS* global_costmap_;
        costmap_2d::Costmap2DROS* local_costmap_;

        //semantic_planner::SemanticPlannerGlobal* planner_; 
        //semantic_planner::SemanticPlannerLocal* controller_;
        semantic_planner::SemanticPlannerGlobal* planner_;
        semantic_planner::SemanticPlannerLocal* controller_;
        semantic_planner::GoalMonitor* goal_monitor_;

        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

        std::string robot_base_frame_, global_frame_;

        double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;

        //set up plan triple buffer
      	std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      	std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      	std::vector<geometry_msgs::PoseStamped>* controller_plan_;

        //set up the planner's thread
        bool runPlanner_, runController_;
        boost::mutex planner_mutex_;
        boost::condition_variable planner_cond_;
        geometry_msgs::PoseStamped planner_goal_;
        boost::thread* planner_thread_;


        bool new_global_plan_;


        geometry_msgs::PoseStamped oscillation_pose_;
        ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
        MoveRobotState state_;
        
        double oscillation_timeout_, oscillation_distance_;
        double planner_patience_, controller_patience_;

        boost::recursive_mutex configuration_mutex_;
        bool setup_, p_freq_change_, c_freq_change_;

        ros::Publisher vel_pub_, action_goal_pub_;
        ros::Subscriber goal_sub_, constraints_sub_; 

        ros::ServiceServer make_plan_srv_;  

        sem_nav_msgs::SemanticPose robot_pose; 

        sem_nav_msgs::Constraints constraints_;   


	};
};
#endif
