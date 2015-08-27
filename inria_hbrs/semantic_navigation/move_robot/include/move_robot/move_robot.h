/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Niranjan Deshpande
*********************************************************************/

#ifndef MOVE_ROBOT_ACTION_H_
#define MOVE_ROBOT_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_robot/MoveRobotConfig.h"

namespace move_robot 
{
	//typedefs to help us out with the action server so that we don't hace to type so much
	typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveRobotActionServer;

	enum MoveBaseState {
      PLANNING,
      CONTROLLING,
      STOP
    };

    /*
     * @class MoveRobot
     * @brief A class that uses the actionlib::ActionServer interface to move the robot to a geometric goal location.
     */
	class MoveRobot 
  {
	public:
	    /*
         * @brief  Constructor for the MoveRobot object
         * @param name The name of the class
         * @param tf A reference to a TransformListener
          @param planner_costmap A pointer to the global semantic costmap to be used by global planner
         */
		MoveRobot(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap);

		/*
         * @brief  Destructor - Cleans up
         */
		virtual ~MoveRobot();

         /*
          * @brief  Initialization function for the MoveRobot object
          * @param  tf A reference to a TransformListener
          * @param  planner_costmap A pointer to the global costmap to use for global planning
          */
		void initialize(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap);

		/*
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /*
         * @brief  Performs a control cycle
         * @param goal A reference to the goal to pursue
         * @param global_plan A reference to the global plan being used
         * @return True if processing of the goal is done, false otherwise
         */
        bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);
        
	protected:
		bool initialized_;
		ros::Publisher plan_publisher_;
		
	private:

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

        /*
         * @brief  Publish a velocity command of zero to the base
         */
        void publishZeroVelocity();

        void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

        /*
         * @brief  Reset the state of the move_robot action and send a zero velocity command to the base
         */
        void resetState();

        double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

        bool isQuaternionValid(const geometry_msgs::Quaternion& q);
		
		geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

		void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
		
		tf::TransformListener* tf_;

        MoveRobotActionServer* as_;

        /**
         * @brief To store copies of global and local costmaps
         */
        costmap_2d::Costmap2DROS* planner_costmap_, *controller_costmap_;

        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        boost::shared_ptr<nav_core::BaseLocalPlanner> controller_;

        std::string robot_base_frame_, global_frame_;

        ros::Publisher action_goal_pub_, vel_pub_;
        ros::Subscriber goal_sub_;

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

        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

        bool new_global_plan_;


        geometry_msgs::PoseStamped oscillation_pose_;
        ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
        MoveBaseState state_;
        
        double oscillation_timeout_, oscillation_distance_;
        double planner_patience_, controller_patience_;

        boost::recursive_mutex configuration_mutex_;
        bool setup_, p_freq_change_, c_freq_change_;
        ros::Publisher current_goal_pub_;
        bool goal_reached;


	};

};
#endif
