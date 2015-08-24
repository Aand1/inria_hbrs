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

#include <move_robot/move_robot.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>

namespace move_robot 
{
    MoveRobot::MoveRobot(tf::TransformListener& tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap) :
      tf_(tf),
      as_(NULL),
      planner_costmap_(NULL),
      controller_costmap_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      runPlanner_(false),
      planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
      initialized_(false)
    {
      initialize(planner_costmap, controller_costmap);        

    }

	MoveRobot::~MoveRobot() 
	{
    if(as_ != NULL)
      delete as_;

		
	}

  
  bool MoveRobot::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {

    if(!controller_->setPlan(*latest_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

  }

  void MoveRobot::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_robot_goal)
  { 
    
    if(!isQuaternionValid(move_robot_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_robot_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::NodeHandle n;
    while(n.ok())
    {
      
      if(as_->isPreemptRequested())
      {
        if(as_->isNewGoalAvailable())
        {
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          ROS_INFO_STREAM("Got a goal");
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_->getGlobalFrameID())
      {
        ROS_INFO_STREAM("Got a goal");
        goal = goalToGlobalFrame(goal);

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

      }

      //the real work on pursuing a goal using local planner
      bool done = executeCycle(goal, global_plan);

      //if the node is killed then we'll abort and return
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
      return;

    }

  }

  
  

	
};
