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

#ifndef PUSH_ACTION_H_
#define PUSH_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_object_actions/PushAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <move_object_actions/push_planner.h>
#include <global_planner/planner_core.h>

using std::string;

namespace move_object
{
	//typedefs to help us out with the action server so that we don't hace to type so much
	typedef actionlib::SimpleActionServer<move_object_actions::PushAction> PushActionServer;

  enum PushActionState 
  {
      PLAN,
      CONTROL,
      STOP
  };

	
	class PushAction 
	{
	public:
		  PushAction(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap);

      virtual ~PushAction();

      /*
       * @brief  Initialization function for the MoveRobot object
       * @param  tf A reference to a TransformListener
       * @param  planner_costmap A pointer to the global costmap to use for global planning
       */
		  void initialize(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap);

      void executeCb(const move_object_actions::PushGoalConstPtr &goal);

  private:

      bool planningCycle(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool controlCycle();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

    	PushActionServer* as_;
    	//MoveRobotActionServer* as_;
        //actionlib::SimpleActionServer<move_object_msgs::PushAction> as_;

    	//boost::shared_ptr<move_object::PushPlanner> planner_;

    	tf::TransformListener* tf_;

    	/*
       * @brief To store copies of global and local costmaps
       */
      costmap_2d::Costmap2DROS* planner_costmap_, *controller_costmap_;

    	bool initialized_;

    	ros::NodeHandle nh_;

      //Publishers
      ros::Publisher action_goal_pub_;
      //Subscribers
      ros::Subscriber goal_sub_;
      
      PushActionState state_;

      //Parameters for global planner
      geometry_msgs::PoseStamped planner_goal_;
      global_planner::GlobalPlanner* planner_;

      std::string robot_base_frame_, global_frame_;
      double planner_frequency_, planner_patience_;
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;


};





};
#endif
