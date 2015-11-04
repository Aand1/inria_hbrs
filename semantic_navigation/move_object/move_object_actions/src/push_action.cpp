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

#include <move_object_actions/push_action.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace move_object
{
	PushAction::PushAction(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap) :
        as_(NULL),
        initialized_(false),
        planner_costmap_(NULL),
        planner_(NULL)        
    {
    	initialize(tf, planner_costmap, controller_costmap);
    	
    	//as_ = new MoveRobotActionServer(ros::NodeHandle(), "move_robot", boost::bind(&PushObject::executeCb, this, _1), false);

        


    }

    PushAction::~PushAction() 
	{
        if(as_ != NULL)
            delete as_;

        if(planner_costmap_ != NULL)
      		delete planner_costmap_;

      	delete planner_plan_;
        delete latest_plan_;
      
		
	}

	void PushAction::initialize(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap)
    {
        if(!initialized_)
        {
            ros::NodeHandle private_nh("~");
            ros::NodeHandle nh;

            planner_costmap_ = planner_costmap;
            tf_ = tf;

            robot_base_frame_ = std::string("base_link");
            global_frame_ = std::string("/map");
            planner_frequency_ = 10.0;
            planner_patience_ = 5.0;

            //set up plan triple buffer
    		planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    		latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    		//to send goals as PoseStamped messages over a topic like nav_view and rviz
    		ros::NodeHandle simple_nh;
    		goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, boost::bind(&PushAction::goalCB, this, _1));

    		ros::NodeHandle action_nh("push_action");
    		action_goal_pub_ = action_nh.advertise<move_object_actions::PushActionGoal>("goal", 1);

    		//initialize the global planner
    		planner_ = new global_planner::GlobalPlanner("push_action/global_planner", planner_costmap->getCostmap(), global_frame_);

    		as_ = new PushActionServer(ros::NodeHandle(), "push_action", boost::bind(&PushAction::executeCb, this, _1), false);

    		as_->start();

    		initialized_ = false;


            

        }
    }


    void PushAction::executeCb(const move_object_actions::PushGoalConstPtr& goal)
    {
    	if(!isQuaternionValid(goal->target_pose.pose.orientation))
    	{
            as_->setAborted(move_object_actions::PushResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }

        geometry_msgs::PoseStamped goal_ = goalToGlobalFrame(goal->target_pose);

        planner_goal_ = goal_;

        state_ = PLAN;

        ros::NodeHandle n;

        while(n.ok())
        {
        	//accept new goal
        	if(as_->isNewGoalAvailable())
            {
            	ROS_INFO_STREAM("Gaol received");
            	move_object_actions::PushGoal new_goal = *as_->acceptNewGoal();

            	if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
    			{
            		as_->setAborted(move_object_actions::PushResult(), "Aborting on goal because it was sent with an invalid quaternion");
            		return;
        		}

        		geometry_msgs::PoseStamped new_goal_ = goalToGlobalFrame(new_goal.target_pose);

        		planner_goal_ = new_goal_;

            }

            switch(state_)
        	{
                case PLAN:
                {
                	planningCycle(planner_goal_, *planner_plan_);

                }
                break;

                case CONTROL:
                {
                	controlCycle();

                }
                break;

                case STOP:
                {

                }
                break;

        	}

        }

    }

    bool PushAction::planningCycle(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
    	plan.clear();

    	//get the starting pose of the robot
        tf::Stamped<tf::Pose> global_pose;
        if(!planner_costmap_->getRobotPose(global_pose)) 
        {
          ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
          return false;
        }

        geometry_msgs::PoseStamped start;
        tf::poseStampedTFToMsg(global_pose, start);

        //if the planner fails or returns a zero length plan, planning failed
        if(!planner_->makePlan(start, goal, plan) || plan.empty())
        {
          ROS_DEBUG_NAMED("push_action","Failed to find a plan to move the object to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
          return false;
        }

    	return true;
    }

    bool PushAction::controlCycle()
    {

    	ROS_INFO_STREAM("Running controller");
    	return true;
    }


    //Miscellenaous functions
    void PushAction::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
	    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
	    
	    move_base_msgs::MoveBaseActionGoal action_goal;
	    action_goal.header.stamp = ros::Time::now();
	    action_goal.goal.target_pose = *goal;

	    action_goal_pub_.publish(action_goal);
    }

    bool PushAction::isQuaternionValid(const geometry_msgs::Quaternion& q)
    {
    	//first we need to check if the quaternion has nan's or infs
	    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
	    {
	        ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
	        return false;
	    }

	    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

	    //next, we need to check if the length of the quaternion is close to zero
	    if(tf_q.length2() < 1e-6)
	    {
	        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
	        return false;
	    }

	    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
	    tf_q.normalize();

	    tf::Vector3 up(0, 0, 1);

	    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

	    if(fabs(dot - 1) > 1e-3)
	    {
	        ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
	        return false;
	    }

	    return true;
    }

    geometry_msgs::PoseStamped PushAction::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
    {
	    std::string global_frame = planner_costmap_->getGlobalFrameID();
     	tf::Stamped<tf::Pose> goal_pose, global_pose;
  	    poseStampedMsgToTF(goal_pose_msg, goal_pose);

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = ros::Time();

        try
        {
            tf_->transformPose(global_frame, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex)
        {
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s", goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_pose, global_pose_msg);
        return global_pose_msg;
    }


};
