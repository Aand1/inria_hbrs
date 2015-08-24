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

  bool MoveRobot::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_robot","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;


  }

  void MoveRobot::planThread()
  {
    ROS_INFO_STREAM("Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;

    boost::unique_lock<boost::mutex> lock(planner_mutex_);

    while(n.ok())
    {
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_)
      {
        //if we should not be running the planner then suspend this thread
        //ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }

      ros::Time start_time = ros::Time::now();

      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();

      //run planner
      planner_plan_->clear();

      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan)
      {
        //std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        temp_plan = planner_plan_;
        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        lock.unlock();
        
        //publish the plan for visualization purposes
        publishPlan(*latest_plan_);
      }

      //take the mutex for the next iteration
      lock.lock();
      ROS_INFO_STREAM(planner_frequency_);
      //setup sleep interface for the planner
      if(planner_frequency_ > 0)
      {
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0))
        {
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveRobot::wakePlanner, this);
        }
      }

    }

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
