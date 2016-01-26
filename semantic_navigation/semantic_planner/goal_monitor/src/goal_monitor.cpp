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

#include "goal_monitor/goal_monitor.h"

using namespace semantic_planner;

GoalMonitor::GoalMonitor() :
	new_goal_(false)
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	goal_reached_publisher = private_nh.advertise<std_msgs::Bool>("goal_reached", 10);

	//subscriber for goals as move_object_actions::Goal messages over a topic
    ros::NodeHandle semantic_planner_global_nh("semantic_planner_global");
    //goal_sub_ = semantic_planner_global_nh.subscribe<move_object_actions::PushObjectGoals>("goal", 1, boost::bind(&GoalMonitor::goalCb, this, _1));

    ros::NodeHandle move_base_nh("move_base_simple");
    move_base_goal_sub_ = move_base_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GoalMonitor::goalCB, this, _1));

    semantic_localization = nh.subscribe<sem_nav_msgs::SemanticPose>("semantic_localization", 1, boost::bind(&GoalMonitor::localizationCb, this, _1));
	
	goal_reached.data = false;
}

GoalMonitor::~GoalMonitor()
{
	
}

void GoalMonitor::goalReached()
{
	//ROS_INFO_STREAM("monitor running");
	if (goal_reached.data == false && new_goal_ == true)
	{
		//double goal_x = global_goal.pose.position.x;
       // double goal_y = global_goal.pose.position.y;

        //double yaw = tf::getYaw(global_goal.pose.orientation.w);
        //ROS_INFO_STREAM("monitor running");
        //ROS_INFO_STREAM("global_pose:" << global_pose);
        tf::Stamped<tf::Pose> goal_point;
        geometry_msgs::PoseStamped pose = global_pose.geometric_pose;

        tf::poseStampedMsgToTF(global_goal, goal_point);
        //tf::poseStampedMsgToTF(global_pose.geometric_pose, global_pose);
    
        double goal_x = goal_point.getOrigin().getX();
        double goal_y = goal_point.getOrigin().getY();
        double yaw = tf::getYaw(goal_point.getRotation());

        double goal_th = yaw;
        

        //ROS_INFO_STREAM("global_pose:" << global_pose);
        //ROS_INFO_STREAM("goal_x:" << goal_x);
        //ROS_INFO_STREAM("goal_y:" << goal_y);
        //ROS_INFO_STREAM("goal_th:" << yaw);

	    


		if ((getGoalPositionDistance(pose, goal_x, goal_y) <= 0.3)) 
		{
			double distance = getGoalPositionDistance(pose, goal_x, goal_y);

		    double angle = getGoalOrientationAngleDifference(global_goal, goal_th);

		    //ROS_INFO_STREAM("angle:" << fabs(angle));

			if (fabs(angle) <= 2.0) 
			{
				goal_reached.data = true;

				goalReachedpublisher();

				new_goal_ == false;
			}

		}

		else
		{
			//ROS_INFO_STREAM("Goal not yet reached");
		}

    

	}
	
}

bool GoalMonitor::goalMonitor(const geometry_msgs::PoseStamped& goal, double tolerance)
{
	tf::Stamped<tf::Pose> goal_point;
    geometry_msgs::PoseStamped pose = global_pose.geometric_pose;

    tf::poseStampedMsgToTF(goal, goal_point);
    
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();
    double yaw = tf::getYaw(goal_point.getRotation());
    double goal_th = yaw;


    if ( (getGoalPositionDistance(pose, goal_x, goal_y) <= tolerance) ) 
	{
		ROS_INFO_STREAM("Goal reached");

	/*	double angle = getGoalOrientationAngleDifference(global_goal, goal_th);

		if (fabs(angle) <= 2.0) 
		{
			return true;
		
		}*/

		return true;

	}

	else
		return false;
        
}

/*void GoalMonitor::goalCb(const move_object_actions::PushObjectGoals::ConstPtr& goal)
{
	//move_object_actions::PushObjectGoals goal_temp = *goal;
	//global_goal = goal_temp.push_object_goal;

	global_goal = (*goal).push_object_goal;
	
	goal_reached.data = false;
	new_goal_ = true;

	//ROS_INFO_STREAM(global_goal);
}*/

void GoalMonitor::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	global_goal = *goal;

	goal_reached.data = false;
	new_goal_ = true;

	//ROS_INFO_STREAM(global_goal);

}


void GoalMonitor::localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose)
{
	global_pose = *pose;
	global_pose.geometric_pose;

	//ROS_INFO_STREAM("Localization received");
	//ROS_INFO_STREAM("global_pose:" << global_pose);

	
}

double GoalMonitor::getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y) 
{
	//ROS_INFO_STREAM("goal_x:" << goal_x);
	//ROS_INFO_STREAM("global_pose.pose.position.x:" << global_pose.pose.position.x);
	//ROS_INFO_STREAM("goal_y:" << goal_y);
	//ROS_INFO_STREAM("global_pose.pose.position.y:" << global_pose.pose.position.y);
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
}

double GoalMonitor::getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th) 
{

    double yaw = global_pose.pose.orientation.w;

    return angles::shortest_angular_distance(yaw, goal_th);
}

void GoalMonitor::goalReachedpublisher()
{
	//goal_reached.publish(reached_goal_);
	ROS_INFO_STREAM("Goal reached");

	while(goal_reached_publisher.getNumSubscribers()==0)
    {
        
    }

	//ROS_ERROR("Got subscriber");

	goal_reached_publisher.publish(goal_reached);
}