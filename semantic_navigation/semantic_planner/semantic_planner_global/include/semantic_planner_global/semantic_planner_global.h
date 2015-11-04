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

#ifndef _SEMANTIC_PLANNER_GLOBAL_
#define _SEMANTIC_PLANNER_GLOBAL_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h> 
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>
#include <dynamic_reconfigure/server.h>
#include <semantic_planner_global/SemanticPlannerGlobalConfig.h>
#include <std_msgs/Bool.h>
#include <move_object_actions/PushObjectGoals.h>


namespace semantic_planner {

	class Expander;
    class GridPath;
	
	class SemanticPlannerGlobal : public nav_core::BaseGlobalPlanner
	{
	public:
		/**
         * @brief  Default constructor for the SemanticPlannerGlobal object
         */
		SemanticPlannerGlobal();

		/**
         * @brief  Constructor for the SemanticPlannerGlobal object
         * @param  tf A reference to a TransformListener
         */
        SemanticPlannerGlobal(tf::TransformListener* tf);

		/**
         * @brief  Default deconstructor for the SemanticPlannerGlobal object
         */
         ~SemanticPlannerGlobal();


        /**
         * @brief  Initialization function for the SemanticPlannerGlobal object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		void initialize(tf::TransformListener* tf);

		/**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
		bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		/**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

         /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

	protected:
		costmap_2d::Costmap2DROS* global_costmap_;
		tf::TransformListener* tf_;

	    global_planner::PotentialCalculator* p_calc_;
	    global_planner::Expander* planner_ ;
	    global_planner::Traceback* path_maker_;

		std::string frame_id_;
		ros::Publisher publish_plan_, publish_plan_visualization_;
		bool initialized_, allow_unknown_;

		std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        std::vector<geometry_msgs::PoseStamped>* latest_plan_;

	private:
		void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        bool executeCb(const std_msgs::BoolConstPtr& run_planner);
        void reset();
        //void goalCb(const move_object_actions::Goal::ConstPtr& goal);
        void goalCb(const move_object_actions::PushObjectGoals::ConstPtr& goal);
        void triggerCb(const std_msgs::Bool::ConstPtr& trigger);
        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

		boost::mutex mutex_;
		double planner_window_x_, planner_window_y_, default_tolerance_;
		std::string tf_prefix_;
		int publish_scale_;
		float* potential_array_;
		bool new_goal_;

		ros::Publisher plan_pub_, plan_for_local_planner;
		ros::Subscriber goal_sub_;
		ros::Subscriber trigger_sub_;
		ros::Subscriber pose_sub_;

		geometry_msgs::PoseStamped object_goal_pose_;

		float convert_offset_;

		dynamic_reconfigure::Server<semantic_planner_global::SemanticPlannerGlobalConfig> *dsrv_;
		void reconfigureCB(semantic_planner_global::SemanticPlannerGlobalConfig &config, uint32_t level);

			
	};


}
#endif