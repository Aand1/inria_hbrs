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
	void MoveRobot::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
	    ROS_INFO_STREAM("In ROS goal callback");
	    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
	    move_base_msgs::MoveBaseActionGoal action_goal;
	    action_goal.header.stamp = ros::Time::now();
	    action_goal.goal.target_pose = *goal;

	    action_goal_pub_.publish(action_goal);
    }

    bool MoveRobot::isQuaternionValid(const geometry_msgs::Quaternion& q)
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


	geometry_msgs::PoseStamped MoveRobot::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
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