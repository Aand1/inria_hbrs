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

namespace move_robot 
{
    MoveRobot::MoveRobot(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap) :
      as_(NULL),
      planner_costmap_(NULL),
      controller_costmap_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      runPlanner_(false), runController_(false),
      planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
      initialized_(false),
      new_global_plan_(false),
      c_freq_change_(false),
      goal_reached(false)
    {
      initialize(tf, planner_costmap, controller_costmap);        

    }

	MoveRobot::~MoveRobot() 
	{
    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ != NULL)
      delete planner_costmap_;

    if(controller_costmap_ != NULL)
      delete controller_costmap_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    controller_.reset();

		
	}


  void MoveRobot::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_robot_goal)
  { 
    //ROS_INFO_STREAM("In executecb");    
    if(!isQuaternionValid(move_robot_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_robot_goal->target_pose);
    current_goal_pub_.publish(goal);

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
            //ROS_INFO_STREAM("Got a goal");
            goal = goalToGlobalFrame(new_goal.target_pose);

            //Reset state for the next execution cycle
            state_ = PLANNING;


          }

          else 
          {
            //if we've been preempted explicitly we need to shut things down
            resetState();

            //notify the ActionServer that we've successfully preempted
            ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
            as_->setPreempted();

            //we'll actually return from execute after preempting
            return;
          }

      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_->getGlobalFrameID())
      {
          //ROS_INFO_STREAM("Got a goal");
          goal = goalToGlobalFrame(goal);

          state_ = PLANNING;

      /*  //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();*/

      }

      if(new_global_plan_)
      {
        state_ = CONTROLLING;
      }

      switch(state_)
      {
          case PLANNING:
          {
              //we have a goal so start the planner
              boost::unique_lock<boost::mutex> lock(planner_mutex_);
              planner_goal_ = goal;
              runPlanner_ = true;
              planner_cond_.notify_one();
              lock.unlock();
          }
          break;

          case CONTROLLING:
          {
              bool done = executeCycle(goal, global_plan);

              if(done)
              {
                  return;
              }

          }
          break;

          case WAITING:
          {
              resetState();

              //disable the planner thread
              boost::unique_lock<boost::mutex> lock(planner_mutex_);
              runPlanner_ = false;
              lock.unlock();
              

          }
          break;

      }

    



    /*  //the real work on pursuing a goal using local planner   
      bool done = executeCycle(goal, global_plan);
      

      if(done)
          return;

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_))
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());*/

    }

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    resetState();
    return;

  }

  void MoveRobot::resetState()
  {
      // Disable the planner thread
      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      runPlanner_ = false;
      state_ = PLANNING;
      lock.unlock();

      publishZeroVelocity();
  }


/*  void MoveRobot::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    
    

    

    ros::NodeHandle n;
    while(n.ok())
    {
      

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          //current_goal_pub_.publish(goal);

          
        }
        
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        //current_goal_pub_.publish(goal);

        
      }

      if(new_global_plan_)
      {
        state_ = CONTROLLING;
      }

      if(goal_reached == true)
      {
        state_ = STOP;
      }

      //for timing that gives real time even in simulation
      //ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      //bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      //if(done)
      //  return;

      switch(state_)
      {
        case PLANNING:
        {
          boost::mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        //ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

        case STOP:

        break;

        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING:
          bool done = executeCycle(goal, global_plan);
          if(done)
            return;

        break;

        
        






      }

      
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  } */


  
  

	
};
