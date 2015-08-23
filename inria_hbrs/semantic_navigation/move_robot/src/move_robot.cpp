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
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>

namespace move_robot 
{
    MoveRobot::MoveRobot(tf::TransformListener& tf, costmap_2d::Costmap2DROS* planner_costmap) :
      tf_(tf),
      as_(NULL),
      planner_costmap_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      runPlanner_(false)
    {
      initialize(planner_costmap);        

    }

	MoveRobot::~MoveRobot() 
	{
    if(as_ != NULL)
      delete as_;

		
	}

  void MoveRobot::initialize(costmap_2d::Costmap2DROS* planner_costmap)
  {
    //if(!initialized_)
    {
      ros::NodeHandle private_nh("~");
      ros::NodeHandle nh;

      planner_costmap_ = planner_costmap;

      as_ = new MoveRobotActionServer(ros::NodeHandle(), "move_robot", boost::bind(&MoveRobot::executeCb, this, _1), false);

      ros::NodeHandle action_nh("move_robot");
      action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

      ros::NodeHandle simple_nh("move_base_simple");
      goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveRobot::goalCB, this, _1));

      plan_publisher_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      //initializing some parameters that will be global to the move robot 
      std::string global_planner, local_planner;
      private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));

      //initialize the global planner
      try 
      {
        planner_ = bgp_loader_.createInstance(global_planner);
        planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_);
      } catch (const pluginlib::PluginlibException& ex) 
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(1);
      }

      //set up plan buffer
      planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
      //set up the planner's thread
      planner_thread_ = new boost::thread(boost::bind(&MoveRobot::planThread, this));

      //we're all set up now so we can start the action server
      as_->start();


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
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    else
    {
      //ROS_INFO_STREAM("Got a plan");
    }

    return true;


  }

  void MoveRobot::planThread()
  {
    ROS_INFO_STREAM("Starting planner thread...");
    ros::NodeHandle n;
    //boost::unique_lock<boost::mutex> lock(planner_mutex_);

    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    lock.unlock();

    while(n.ok())
    {
      //check if we should run the planner
      while(!runPlanner_)
      {
        //if we should not be running the planner then suspend this thread
        //ROS_INFO_STREAM("Planner thread is suspending");
        //planner_cond_.wait(lock);
        
      }
      ROS_INFO_STREAM("Planner thread running");
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      //lock.unlock();

      //run planner
      planner_plan_->clear();

      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan)
      {
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        
        //publish the plan for visualization purposes
        publishPlan(*temp_plan);
      }
    }

  }

  void MoveRobot::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_robot_goal)
  { 
    ROS_INFO_STREAM("Got a goal");
    if(!isQuaternionValid(move_robot_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    ROS_INFO_STREAM("Got a goal");
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_robot_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    ros::NodeHandle n;
    while(n.ok())
    {
      
      if(as_->isPreemptRequested())
      {
        if(as_->isNewGoalAvailable())
        {
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          ROS_INFO_STREAM("Got a goal");
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_->getGlobalFrameID())
      {
        ROS_INFO_STREAM("Got a goal");
        goal = goalToGlobalFrame(goal);

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

      }

      //if the node is killed then we'll abort and return
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
      return;

    }



  }

  void MoveRobot::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    ROS_INFO_STREAM("In ROS goal callback");
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  

	
};
