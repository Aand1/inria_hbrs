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
    void MoveRobot::publishZeroVelocity()
    {

	    geometry_msgs::Twist cmd_vel;
	    cmd_vel.linear.x = 0.0;
	    cmd_vel.linear.y = 0.0;
	    cmd_vel.angular.z = 0.0;
	    vel_pub_.publish(cmd_vel);

    }

    
    bool MoveRobot::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
    
	    //we need to be able to publish velocity commands
	    geometry_msgs::Twist cmd_vel;

	    
	    //check that the observation buffers for the costmap are current, we don't want to drive blind
	    if(!controller_costmap_->isCurrent())
	    {
		     ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
		     publishZeroVelocity();
		     return false;
	    }
    
    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_)
    {
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
	  
        
        if(!controller_->setPlan(*controller_plan_))
        {
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
            resetState();

            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
            return true;
        }

      
    }

    
    //check to see if we've reached our goal
    if(controller_->isGoalReached())
    {

        ROS_INFO_STREAM("move_base, Goal reached!");
        resetState();

          //disable the planner thread
          //boost::unique_lock<boost::mutex> lock(planner_mutex_);
          //runPlanner_ = false;
          //lock.unlock();
          goal_reached = true;
          state_ = STOP;

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
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

        else{
        	state_ = PLANNING;
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