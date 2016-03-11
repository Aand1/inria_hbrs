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

#include "semantic_planner_local/semantic_planner_local.h"
#include <geometry_msgs/Twist.h>

using std::vector;
using namespace semantic_planner;

SemanticPlannerLocal::SemanticPlannerLocal(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
//SemanticPlannerLocal::SemanticPlannerLocal(tf::TransformListener* tf) :
    tf_(tf),
    local_costmap_(costmap_ros), 
    initialized_(false),
    controller_plan_(NULL)
{
   	//initialize the controller
   	//initialize(name, tf , nh, costmap_ros);
    //initialize(tf);

      ros::NodeHandle nh;
      ros::NodeHandle private_nh("~");

      //Get some parameters for the local controller and its costmap
      std::string local_planner;
      private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
      private_nh.param("controller_frequency", controller_frequency_, 20.0);
      private_nh.param("controller_patience", controller_patience_, 15.0);

      controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

      tp.initialize("TrajectoryPlannerROS", tf_, local_costmap_);

      //for comanding the base
      vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


SemanticPlannerLocal::~SemanticPlannerLocal()
{

}


//void SemanticPlannerLocal::initialize(std::string name, tf::TransformListener* tf, ros::NodeHandle &nh, costmap_2d::Costmap2DROS* costmap_ros)
void SemanticPlannerLocal::initialize(tf::TransformListener* tf)
{
/*    if (!initialized_) 
    {
        
        ros::NodeHandle private_nh("~");
        tf_ = tf;
        ros::NodeHandle nh;
        //nh_ = nh;
        //local_costmap_ = costmap_ros;

        //Get some parameters for the local controller and its costmap
        std::string local_planner;
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);

        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);

        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        //subscriber for plans as geometry_msgs::PoseStamped messages over a topic
        ros::NodeHandle simple_nh("semantic_planner_local");
        ros::NodeHandle plan_nh("semantic_planner_global");
        plan_sub_ = plan_nh.subscribe<nav_msgs::Path>("global_plan", 1, boost::bind(&SemanticPlannerLocal::planCb, this, _1) );
        trigger_sub_ = simple_nh.subscribe<std_msgs::Bool>("trigger", 1, boost::bind(&SemanticPlannerLocal::triggerCb, this, _1));

    
        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
        local_costmap_->start();

        tp.initialize("TrajectoryPlannerROS", tf_, local_costmap_);

        //tc_ = new nav_core::BaseLocalPlanner();

        //for comanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


        initialized_ = true;

    }*/
	
}

void SemanticPlannerLocal::planCb(const nav_msgs::Path::ConstPtr& plan)
{
    *controller_plan_ = (*plan).poses;
    //std::vector<geometry_msgs::PoseStamped>* temp_plan;

    //*temp_plan = (*plan).poses;

    //ROS_INFO_STREAM("plan_received");

    //controller_plan_ = temp_plan;
    
    if (tp.setPlan(*controller_plan_) )
    {
        new_global_plan_ = true;
    }

}


void SemanticPlannerLocal::triggerCb(const std_msgs::Bool::ConstPtr& trigger)
{ 
    geometry_msgs::Twist cmd_vel;

    if (trigger->data == true)
    {
        if (new_global_plan_ == true)
        {
            ROS_INFO_STREAM("Trigger to local planner");
            //we need to be able to publish velocity commands
            

            //ros::NodeHandle n;
            //while(n.ok())
            //{
            if(tp.computeVelocityCommands(cmd_vel))
            {
                //make sure that we send the velocity command to the base
                vel_pub_.publish(cmd_vel);
                ROS_INFO_STREAM(cmd_vel);
            }
            //}    
        }
    }

    else if (trigger->data == false)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        vel_pub_.publish(cmd_vel);
    }

}

bool SemanticPlannerLocal::sendVelocityCommands()
{ 
    geometry_msgs::Twist cmd_vel;

    if(tp.computeVelocityCommands(cmd_vel))
    {
        vel_pub_.publish(cmd_vel);

        return true;
    }

    else
        return false;

}

void SemanticPlannerLocal::stop()
{ 
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    
    vel_pub_.publish(cmd_vel);
}

void SemanticPlannerLocal::back()
{ 
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = -0.30;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    
    vel_pub_.publish(cmd_vel);

}

void SemanticPlannerLocal::setParam(sem_nav_msgs::LocalPlannerConstraints& constraints)
{

    for (int i = 0; i < constraints.doubles.size(); i++)
    {

        if (constraints.doubles[i].name.compare("max_vel_x")  == 0 )
        {
            conf.doubles.push_back(constraints.doubles[i]);
        }

        srv_req.config = conf;

        ros::service::call("/push_action/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);

    }

}


