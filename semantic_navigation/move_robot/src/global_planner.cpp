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
#include <boost/thread.hpp>


namespace move_robot 
{

    void MoveRobot::planThread()
    {
	    
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
	            std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
	            temp_plan = planner_plan_;
	            lock.lock();
	            planner_plan_ = latest_plan_;
        		latest_plan_ = temp_plan;
        		new_global_plan_ = true;
	            lock.unlock();
	        
	            //publish the plan for visualization purposes
	            publishPlan(*latest_plan_);

	            if(runPlanner_)
          		    state_ = CONTROLLING;

	        }

	        //take the mutex for the next iteration
	        lock.lock();
	        	        
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


    void MoveRobot::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) 
    {
	    //create a message for the plan
	    nav_msgs::Path gui_path;
	    gui_path.poses.resize(path.size());

	    if (!path.empty()) 
	    {
	        gui_path.header.frame_id = path[0].header.frame_id;
	        gui_path.header.stamp = path[0].header.stamp;
	    }

	    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
	    for (unsigned int i = 0; i < path.size(); i++) 
	    {
	        gui_path.poses[i] = path[i];
	    }

	    plan_publisher_.publish(gui_path);

	}
	

	void MoveRobot::wakePlanner(const ros::TimerEvent& event)
    {
	    // we have slept long enough for rate
	    planner_cond_.notify_one();
    }


};