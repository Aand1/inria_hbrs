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
    void MoveRobot::initialize(costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap)
    {
        if(!initialized_)
        {
            ros::NodeHandle private_nh("~");
            ros::NodeHandle nh;

            planner_costmap_ = planner_costmap;
            controller_costmap_ = controller_costmap;
            
            as_ = new MoveRobotActionServer(ros::NodeHandle(), "move_robot", boost::bind(&MoveRobot::executeCb, this, _1), false);

            ros::NodeHandle action_nh("move_robot");
            action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

            ros::NodeHandle simple_nh("move_base_simple");
            goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveRobot::goalCB, this, _1));

            plan_publisher_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

            //initializing some parameters that will be global to the move robot 
            std::string global_planner, local_planner;
            private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
            private_nh.param("planner_frequency", planner_frequency_, 1.0);
            private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));

            //initialize the global planner
            try 
            {
                planner_ = bgp_loader_.createInstance(global_planner);
                planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_);
            } 
            catch (const pluginlib::PluginlibException& ex) 
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
                exit(1);
            }

            //create a local planner
            try 
            {
                controller_ = blp_loader_.createInstance(local_planner);
                //ROS_INFO("Created local_planner %s", local_planner.c_str());
                controller_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_);
            } 
            catch (const pluginlib::PluginlibException& ex) 
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
                exit(1);
            }

            //set up plan buffer
            planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            
            //set up the planner's thread
            planner_thread_ = new boost::thread(boost::bind(&MoveRobot::planThread, this));

            //for comanding the base
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

            //we're all set up now so we can start the action server
            as_->start();

            initialized_ = false;
        }
    }

};