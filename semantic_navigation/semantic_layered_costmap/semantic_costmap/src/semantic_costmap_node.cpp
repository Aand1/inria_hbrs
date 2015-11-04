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
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "std_msgs/Char.h"
#include "std_msgs/String.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "semantic_costmap_node");
	ros::NodeHandle nh;
	tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS* planner_costmap_ros, *controller_costmap_ros_;


    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf);

    ros::Publisher pub = nh.advertise<std_msgs::String>("semantic_costmap", 5);
    //std_msgs::StringPtr str(new std_msgs::String);
   // boost::shared_ptr< std_msgs::Char> str1(new std_msgs::Char);
    //str->data;
    //str1 = (planner_costmap_ros->getCostmap());

    //std_msgs::char::ConstPtr costmap_ptr = planner_costmap_ros;

    //ROS_INFO_STREAM(planner_costmap_ros);

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    //controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf);

	//move_robot::MoveRobot move_robot(&tf, planner_costmap_ros, controller_costmap_ros_);
	
	//ros::MultiThreadedSpinner s;	
	ros::spin();
	
	return(0);
}
