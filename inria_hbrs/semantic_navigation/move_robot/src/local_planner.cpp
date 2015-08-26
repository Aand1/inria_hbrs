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
#include <geometry_msgs/Twist.h>

namespace move_robot 
{
/*	bool MoveRobot::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
    	ROS_INFO_STREAM("Running controller");
    	//TO be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //check that the observation buffers for the costmap are current, we don't want to drive blind
       if(!controller_costmap_->isCurrent())
       {
           ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
           //publishZeroVelocity();
           return false;
       }

       //if we have a new plan then grab it and give it to the controller
       if(new_global_plan_)
       {
           //make sure to set the new plan flag to false
           new_global_plan_ = false;

	       //do a pointer swap under mutex
	       std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

	       boost::unique_lock<boost::mutex> lock(planner_mutex_);
	       controller_plan_ = latest_plan_;
	       latest_plan_ = temp_plan;
	       lock.unlock();

	        //Give the global plan to the local planner(controller)
	        if(!controller_->setPlan(*controller_plan_))
	        {

	            resetState();
	            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
	            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
	            return true;
	        }
	    }

	    //check to see if we've reached our goal
        if(controller_->isGoalReached())
        {
	        ROS_DEBUG_NAMED("move_base","Goal reached!");

	        resetState();	          

	        //disable the planner thread
	        boost::unique_lock<boost::mutex> lock(planner_mutex_);
	        runPlanner_ = false;
	        lock.unlock();
  
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            return true;
        }

        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_->getCostmap()->getMutex()));
            //compute velocity commands to go from one pose to the next in global planner
            if(controller_->computeVelocityCommands(cmd_vel))
            {
                //Send the velocity commands to the base 
                vel_pub_.publish(cmd_vel);	
            }
            else
            {
            	boost::unique_lock<boost::mutex> lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();        
            }

        }

        //we aren't done yet
        return false;

    } */


    void MoveRobot::publishZeroVelocity()
    {

	    geometry_msgs::Twist cmd_vel;
	    cmd_vel.linear.x = 0.0;
	    cmd_vel.linear.y = 0.0;
	    cmd_vel.angular.z = 0.0;
	    vel_pub_.publish(cmd_vel);

    }



  bool MoveRobot::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }
    
    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      int size = planner_plan_->size();
	  ROS_INFO_STREAM(size);
	  
      if(!controller_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      
    }

    //the move_base state machine, handles the control logic for navigation
    if(state_ == CONTROLLING){
      
      //if we're controlling, we'll attempt to find valid velocity commands
      
        ROS_DEBUG_NAMED("move_base","In controlling state.");
        ROS_INFO_STREAM("Control state running");

        //check to see if we've reached our goal
        if(controller_->isGoalReached()){

          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_->getCostmap()->getMutex()));
        
        if(controller_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        

      
    }
    

    //we aren't done yet
    return false;
  }

  double MoveRobot::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }



};