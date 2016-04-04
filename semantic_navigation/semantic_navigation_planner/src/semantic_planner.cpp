/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg, Germany
 *                      Inria, France
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Hochschule Bonn-Rhein-Sieg, Germany and Inria, 
 *     France nor the names of its contributors may be used to 
 *     endorse or promote products derived from this software without 
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Niranjan Vilas Deshpande
 *         (niranjan.deshpande187@gmail.com)
 *
 * Supervised by: Sven Schneider (Hochschule Bonn-Rhein-Sieg)
 *                Prof. Dr. Paul G. Ploeger (Hochschule Bonn-Rhein-Sieg)
 *		          Dr. Anne Spalanzani (Inria)
 *********************************************************************/

#include <semantic_navigation_planner/semantic_planner.h>

namespace semantic_navigation_planner
{

	SemanticPlanner::SemanticPlanner(tf::TransformListener& tf) :
		as_(NULL), tf_(tf)
	 
	{ 	
		ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

	    as_ = new SemanticPlannerActionServer(ros::NodeHandle(), "semantic_planner", boost::bind(&SemanticPlanner::executeCb, this, _1), false);
        as_->start();

        //MoveRobotActionClient move_robot_ac("move_robot_action", true); 

        reasoner =  new semantic_navigation_planner::Reasoner();
        best_path = new semantic_navigation_planner::ComputeBestPath(tf_, nh);

        constraints_pub = nh.advertise<sem_nav_msgs::Constraints>("constraints", 1);

        localization_sub_ = nh.subscribe<sem_nav_msgs::SemanticPose>("semantic_localization", 1, boost::bind(&SemanticPlanner::localizationCb, this, _1));

    /*    mr_ac = new MoveRobotActionClient("move_robot_action", true);
        while(!mr_ac->waitForServer(ros::Duration(5.0)))
	    {
	       ROS_INFO("Waiting for the move robot action server to come up");
	    }*/

        pa_ac = new PushActionClient("push_action", true);
        while(!pa_ac->waitForServer(ros::Duration(5.0)))
        {
           ROS_INFO("Waiting for the push action server to come up");
        }

        ta_ac = new TapActionClient("tap_action", true);
        while(!ta_ac->waitForServer(ros::Duration(5.0)))
        {
           ROS_INFO("Waiting for the tap action server to come up");
        }

	}

	SemanticPlanner::~SemanticPlanner() 
	{

	} 

	void SemanticPlanner::initialize() 
	{

	} 

	void SemanticPlanner::executeCb(const sem_nav_msgs::SemanticPlannerGoalConstPtr& goal)
	{
		sem_nav_msgs::SemanticPlannerGoal new_goal = *goal;

        ///////////////////////////////////////////////////////////////////////////
        

        sem_nav_msgs::BestPath bp;
        bp = best_path->computeBestPath(robot_pose.geometric_pose, new_goal.input.geometric_goal);
        ROS_INFO_STREAM(bp);

        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
		//sem_nav_msgs::Constraints constraints;
        
        //std_msgs::String region;
        //region.data = robot_pose.semantic_pose.name;
        //ROS_INFO_STREAM(region.data);
        //ROS_INFO_STREAM(new_goal.input.purpose);
		//reasoner->computeConstraints(new_goal.input.purpose, constraints);
        //reasoner->computeConstraintsBasedOnLocation(region, constraints);

        //ROS_INFO_STREAM(constraints);

		//constraints_pub.publish(constraints);
        ////////////////////////////////////////////////////////////////////////////

        //bool done = executeCycleOne(new_goal.input.geometric_goal); 
        bool done = executeCycleTwo(bp, new_goal.input.geometric_goal);  

        if (done)
        {
        	as_->setSucceeded(sem_nav_msgs::SemanticPlannerResult(), "Semantic Planner accomplished the task.");
        }

        else
        {	
        	as_->setAborted(sem_nav_msgs::SemanticPlannerResult(), "Semantic Planner failed");
        }

		
	}

    bool SemanticPlanner::executeCycleOne(const geometry_msgs::PoseStamped& geometric_goal)
    {
    	sem_nav_msgs::MoveRobotGoal action_goal;
        
        action_goal.target_pose = geometric_goal;

    	//ROS_INFO_STREAM(move_robot_goal);
    	
    	mr_ac->sendGoal(action_goal);

    	mr_ac->waitForResult();

    	if(mr_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	{
    		ROS_INFO_STREAM("Reached goal");
    		//as_->setSucceeded(sem_nav_msgs::SemanticPlannerResult(), "Semantic Planner accomplished the task.");
    		return true;
    	}
          
        else
        {
        	//as_->setAborted(sem_nav_msgs::SemanticPlannerResult(), "Semantic Planner failed");
        	//return false;
            return true;
        } 

        //return false; 
        return true;
    }

    bool SemanticPlanner::executeCycleTwo(const sem_nav_msgs::BestPath& bp, const geometry_msgs::PoseStamped& geometric_goal)
    {
        if ( (bp.category.compare("Box") == 0) ) 
        {
            sem_nav_msgs::PushGoal push_goal;

            push_goal.object_instance.name = bp.instance;
            push_goal.reach_goal = geometric_goal;
                        
            ROS_INFO("Sending trigger to PushAction");

            pa_ac->sendGoal(push_goal);

            pa_ac->waitForResult();

            if(pa_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                return true;
                ROS_INFO("Hooray, the base pushed the object");
            }
          
            else
            {
                ROS_INFO("The base failed to pushed the object");
                return false;
            }
          

        }

        else if ( (bp.category.compare("Ball") == 0) ) 
        {
            sem_nav_msgs::TapGoal tap_goal;

            tap_goal.object_instance.name = bp.instance;
            tap_goal.reach_goal = geometric_goal;
                        
            ROS_INFO("Sending trigger to TapAction");

            ta_ac->sendGoal(tap_goal);

            ta_ac->waitForResult();

            if(ta_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                return true;
                ROS_INFO("Hooray, the base taped the object");
            }
          
            else
            {
                ROS_INFO("The base failed to tap the object");
                return false;
            }
          

        }

        return true;

    }

    void SemanticPlanner::localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose)
    {
        robot_pose = *pose;  

        //ROS_INFO_STREAM(robot_pose); 
    }

}        
	

