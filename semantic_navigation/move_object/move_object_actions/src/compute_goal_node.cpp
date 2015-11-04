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

#include <ros/ros.h>
#include <move_object_actions/PushObjectGoals.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "compute_goal_node");

	ros::NodeHandle n;

	ros::Rate poll_rate(100);

	bool latchOn = 1;

    ros::Publisher goal_pub = n.advertise<move_object_actions::PushObjectGoals>("semantic_planner_global/goal", 20, true);

/*    while (goal_pub.getNumSubscribers()==0)
    {
    	ROS_INFO_STREAM("Waiting for subscriber");

    	//sleep(10);
    }

    ROS_INFO_STREAM("Got subscriber");*/

    move_object_actions::PushObjectGoals latest_goal;

    latest_goal.instance.name = "box-01";
    latest_goal.push_object_goal.header.frame_id = "/map";
    latest_goal.push_object_goal.header.stamp = ros::Time::now();
    latest_goal.push_object_goal.pose.position.x = 2.0;
    latest_goal.push_object_goal.pose.position.y = 0.4;

    latest_goal.push_object_goal.pose.orientation.w = 1.0;

    goal_pub.publish(latest_goal);
    ros::spinOnce();

    while(ros::ok())
    {
       sleep(3);    
    }

    return 0;

}
