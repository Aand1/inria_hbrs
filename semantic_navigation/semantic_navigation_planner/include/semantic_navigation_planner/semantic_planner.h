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

#ifndef SEMANTIC_PLANNER_H_
#define SEMANTIC_PLANNER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h> 
#include <sem_nav_msgs/SemanticPlannerAction.h>
#include <sem_nav_msgs/MoveRobotAction.h>
#include <semantic_navigation_planner/reasoner.h>
#include <semantic_navigation_planner/compute_best_path.h>

#include <move_base_msgs/MoveBaseAction.h> 
#include <sem_nav_msgs/PushAction.h> 
#include <sem_nav_msgs/TapAction.h>  
#include <sem_nav_msgs/SemanticPose.h> 



//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<sem_nav_msgs::SemanticPlannerAction> SemanticPlannerActionServer;

typedef actionlib::SimpleActionClient<sem_nav_msgs::MoveRobotAction> MoveRobotActionClient;
typedef actionlib::SimpleActionClient<sem_nav_msgs::PushAction> PushActionClient;
typedef actionlib::SimpleActionClient<sem_nav_msgs::TapAction> TapActionClient;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace semantic_navigation_planner
{
	class SemanticPlanner 
	{
		public:
			SemanticPlanner(tf::TransformListener& tf); 

	        virtual ~SemanticPlanner();

	        void initialize();

	    private:
	    	void localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose);
	    	//tf::TransformListener& tf_;
	    	void executeCb(const sem_nav_msgs::SemanticPlannerGoalConstPtr& goal);

	    	void publishConstraints(const sem_nav_msgs::Constraints& constraints);

	    	bool executeCycleOne(const geometry_msgs::PoseStamped& geometric_goal);

	    	bool executeCycleTwo(const sem_nav_msgs::BestPath& bp, const geometry_msgs::PoseStamped& geometric_goal);

	    	tf::TransformListener& tf_;

	    	SemanticPlannerActionServer* as_;

	    	semantic_navigation_planner::Reasoner* reasoner;
	    	semantic_navigation_planner::ComputeBestPath* best_path;

	    	// ROS publishers
            ros::Publisher constraints_pub;
            ros::Subscriber localization_sub_; 

            MoveRobotActionClient* mr_ac;
            PushActionClient* pa_ac;
            TapActionClient* ta_ac; 

            sem_nav_msgs::SemanticPose robot_pose; 


	};
};

#endif