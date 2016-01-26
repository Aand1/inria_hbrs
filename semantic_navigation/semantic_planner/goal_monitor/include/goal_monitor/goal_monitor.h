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

#ifndef _GOAL_MONITOR_
#define _GOAL_MONITOR_

#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sem_nav_msgs/SemanticPose.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>




namespace semantic_planner
{

	class GoalMonitor
	{
	public:

		GoalMonitor();
		
		~GoalMonitor();

		void goalReached();

		bool goalMonitor(const geometry_msgs::PoseStamped& goal, double tolerance);

		void goalReachedpublisher();

	private:
	//	void goalCb(const move_object_actions::PushObjectGoals::ConstPtr& goal);
		void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
		void localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose);
		double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y);
		double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th); 

		geometry_msgs::PoseStamped global_goal;
		bool new_goal_;
		std_msgs::Bool goal_reached;
		sem_nav_msgs::SemanticPose global_pose;

		ros::Publisher goal_reached_publisher;
		ros::Subscriber semantic_localization;
		ros::Subscriber goal_sub_;
		ros::Subscriber move_base_goal_sub_;

	};


};
#endif
