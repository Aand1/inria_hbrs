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

#include <move_object_actions/tap_action.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/DoubleParameter.h>

namespace move_object
{
	TapAction::TapAction(tf::TransformListener& tf) :
        tf_(tf),
        as_(NULL),
        planner_plan_(NULL), latest_plan_(NULL),
        initialized_(false),
        global_costmap_(NULL), local_costmap_(NULL),
        planner_(NULL), tc_(NULL), goal_monitor_(NULL),
        goals(NULL)

    {
    	
        initialize();

    }

    TapAction::~TapAction() 
	{
        if(as_ != NULL)
            delete as_;

        if(global_costmap_ != NULL)
      		delete global_costmap_;

        if(local_costmap_ != NULL)
      		delete local_costmap_;

      	delete planner_plan_;
        delete latest_plan_;
      
		
	}

	void TapAction::initialize()
    {
        if(!initialized_)
        {
            ros::NodeHandle private_nh("~");
            ros::NodeHandle nh;

            global_costmap_ = new semantic_costmap::SemanticCostmapROS("global_costmap", tf_);
            local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);

            planner_ = new semantic_planner::SemanticPlannerGlobal( "global_planner" , global_costmap_);
            tc_ = new semantic_planner::SemanticPlannerLocal( "local_planner", &tf_, local_costmap_ );
            goal_monitor_ = new semantic_planner::GoalMonitor();

            semantic_map_query_ = new semantic_map::SemanticMap(nh);

            goals = new sem_nav_msgs::MoveObjectGoals();

            planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();


            std_msgs::Int32 size;
            size.data = 3;
            std::string name =  "move_object_goals";
            geometry_msgs::Vector3 scale;
            scale.x = scale.y = 0.1;
            scale.z = 0.01;
            goal_marker = new viz_msgs::VisualizationMarker( name, scale, size );

            name = "move_object_paths";
            scale.x = scale.y = 0.025;
            scale.z = 0.01;
            path_marker = new viz_msgs::VisualizationMarker( name, scale, size );

            paths_pub = nh.advertise<visualization_msgs::MarkerArray>("move_object_paths", 1);
            goals_pub = nh.advertise<visualization_msgs::MarkerArray>("move_object_goals", 1);

            //advertise a service for getting a plan
            make_plan_srv_ = private_nh.advertiseService("make_plan", &TapAction::planService, this);


            semantic_localization = nh.subscribe<sem_nav_msgs::SemanticPose>("semantic_localization", 1, boost::bind(&TapAction::localizationCb, this, _1));

            resetState();

            as_ = new TapActionServer(ros::NodeHandle(), "tap_action", boost::bind(&TapAction::executeCb, this, _1), false);
            as_->start();
            global_costmap_->start();
            local_costmap_->start();

    		initialized_ = true;
            

        }
    }


    void TapAction::executeCb(const sem_nav_msgs::TapGoalConstPtr& tap_goal)
    {       
        ROS_INFO_STREAM("Received request for tapping an object:");

        sem_nav_msgs::TapGoal new_goal = *tap_goal;

        object.instance.name = new_goal.object_instance.name;
        semantic_map_query_->getObjectDynamic(object);

        sem_nav_msgs::MoveObjectGoals current_goals;
        
        computeGoals(new_goal.reach_goal, current_goals);

        // initialize the states
        resetState();

        ros::NodeHandle n;

        while(n.ok())
        {
            if(as_->isPreemptRequested())
            {
                if(as_->isNewGoalAvailable())
                {
                    ROS_INFO_STREAM("Received request for pushing an object");

                    sem_nav_msgs::TapGoal new_goal = *as_->acceptNewGoal();
                    object.instance.name = new_goal.object_instance.name;
                    semantic_map_query_->getObjectDynamic(object);

                    computeGoals(new_goal.reach_goal, current_goals);
                    // initialise states
                    resetState();
                                                                
                }
           
                else 
                {
                    //if we've been preempted explicitly we need to shut things down
                    resetState();

                    //notify the ActionServer that we've successfully preempted
                    ROS_DEBUG_NAMED("tap_object","Tap object preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    return;
                }
            }
      
            bool done = executeCycle(current_goals);   

            if (done)
            {
                //clearPublish();
                as_->setSucceeded(sem_nav_msgs::TapResult(), "Goal reached.");
                return;
            }

        }

        //if the node is killed then we'll abort and return
        as_->setAborted(sem_nav_msgs::TapResult(), "Aborting the action because the node has been killed");
        return;
    }

    bool TapAction::executeCycle(const sem_nav_msgs::MoveObjectGoals& goals)
    {    
        switch(tap_action_states_)
        {
            case APPROACHOBJECT:
                {
                    ROS_INFO_STREAM("APPROACHOBJECT");
                    bool approached_object = executeApproachObject(goals.approach_object);

                    if (approached_object)
                    {
                        
                        tap_action_states_ = TAPOBJECT;

                        
                    }

                    else 
                        return false;

                }
                break;


            case TAPOBJECT:
                {
                    ROS_INFO_STREAM("PUSHOBJECT");
                    bool taped_object = executeTapObject(goals.move_object);

                    if (taped_object)
                    {
                        tap_action_states_ = APPROACHGOAL;
                    }
                    else
                        return false;

                }    
                break;

            case APPROACHGOAL:
                {
                    ROS_INFO_STREAM("APPROACHGOAL");
                    bool approached_goal = executeApproachGoal(goals.reach_goal);

                    if (approached_goal)
                    {
                        return true;
                    }
                    else
                        false;

                }    
                break;

        }

        return false;
    }

    bool TapAction::executeApproachObject(const geometry_msgs::PoseStamped& approach_goal)
    {    	

        planner_plan_->clear();

        bool done = false;

        double tolerance = 0.25;
        
        nav_msgs::Path plan;
        
        int index = 0; // index for path publish vizualization message

        move_states_ = PLAN;
       
        while (!done)
        {
            switch(move_states_)
            {
                case PLAN:
                {                        
                    ROS_INFO_STREAM("APPROACHOBJECT: PLAN");
                    bool gotPlan;
                    gotPlan = planner_->makePlanObjectApproach(robot_pose.geometric_pose, approach_goal, *planner_plan_, object);

                    //planner_plan_->erase( planner_plan_->begin()+1, planner_plan_->end() );
                    

                    if (gotPlan)
                    {    
                        //copy the plan into a MoveObjectPaths message
                        plan.poses.resize(planner_plan_->size());
                        
                        for(unsigned int i = 0; i < planner_plan_->size(); ++i)
                        {
                            plan.poses[i] = planner_plan_->at(i);
                        }
                        
                        if( !tc_->setPlan(*planner_plan_) )
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::TapResult(), "Failed to pass approach object global plan to the controller.");
                            done = true;
                            return false;
                        }

                        else
                        {
                            publishPlan(plan, index);
                            move_states_ = CONTROL;
                        }
                    }   

                    else 
                    {
                        as_->setAborted(sem_nav_msgs::TapResult(), "Failed to generate approach object global plans.");
                        resetState();

                        done = true; // Go out of while loop
                        
                        return false;
                    }
                }
                break;

                case CONTROL:
                {
                    ROS_INFO_STREAM("APPROACHOBJECT: CONTROL");
                    if ( !(goal_monitor_->goalMonitor(approach_goal, tolerance)) )
                    {

                        tc_->sendVelocityCommands();
                    }
                    else 
                    {
                        tc_->stop();
                        done = true;
                        return true;
                    }  

                }
                break;
            }
        }

        return false;


    }

    bool TapAction::executeTapObject(const geometry_msgs::PoseStamped& move_object)
    {
        bool done = false;

        double tolerance = 0.1;
        
        nav_msgs::Path plan;
        geometry_msgs::PoseStamped start;
        start.header.frame_id = "/map";
        start.pose = object.geometry.pose;
        
        int index = 1;

        move_states_ = PLAN;
        planner_plan_->clear();

        while (!done)
        {
            planner_plan_->clear();

            switch(move_states_)
            {
                case PLAN:
                {
                    
                    ROS_INFO_STREAM("PUSHOBJECT: Plan");
                    bool gotPlan;
                    gotPlan = planner_->makePlanObjectPush(robot_pose.geometric_pose, move_object, *planner_plan_, object);

                    if (gotPlan)
                    {   
                        //copy the plan into a MoveObjectPaths message
                        plan.poses.resize(planner_plan_->size());
                        for(unsigned int i = 0; i < planner_plan_->size(); ++i)
                        {
                            plan.poses[i] = planner_plan_->at(i);
                        }

                        if( !tc_->setPlan(*planner_plan_) )
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::TapResult(), "Failed to pass move object global plan to the controller.");
                            done = true;
                            return false;
                        }

                        else
                        {
                            publishPlan(plan, index);
                            move_states_ = CONTROL;

                            //Change local controller parameters for object pushing
                            double velocity = 0.8;
                            double sim_time = 3.0;
                            updateControllerParams(velocity, sim_time);
                        }
                    }   

                    else 
                    {
                        as_->setAborted(sem_nav_msgs::TapResult(), "Failed to generate approach object global plans.");
                        resetState();
                        done = true;
                        return false;
                    }
                }

                break;


                case CONTROL:
                {
                    ROS_INFO_STREAM("PUSHOBJECT: Control");


                    if ( !(goal_monitor_->goalMonitor(move_object, tolerance)) )
                    {

                        tc_->sendVelocityCommands();
                    }
                    else 
                    {
                        ros::Duration(2.0).sleep();
                        tc_->stop();

                        // back aside from the object
                        //tc_->back();
                        //ros::Duration(1.5).sleep();
                        //tc_->stop();
                        done = true;
                        return true;
                    } 

                    

                }
                break;
            }

        }
        
        return false;

    }

    bool TapAction::executeApproachGoal(const geometry_msgs::PoseStamped& reach_goal)
    {
        planner_plan_->clear();

        bool done = false;

        double tolerance = 0.1;
        
        nav_msgs::Path plan;
        
        int index = 2; // index for path publish vizualization message

        move_states_ = PLAN;
       
        while (!done)
        {
            switch(move_states_)
            {
                case PLAN:
                {                        
                    ROS_INFO_STREAM("APPROACHGOAL: PLAN");
                    bool gotPlan;
                    gotPlan = planner_->makePlan(robot_pose.geometric_pose, reach_goal, *planner_plan_);

                    

                    if (gotPlan)
                    {    
                        //copy the plan into a MoveObjectPaths message
                        ROS_INFO_STREAM(planner_plan_->size());
                        plan.poses.resize(planner_plan_->size());
                        
                        for(unsigned int i = 0; i < planner_plan_->size(); ++i)
                        {
                            plan.poses[i] = planner_plan_->at(i);
                        }
                        
                        if( !tc_->setPlan(*planner_plan_) )
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::TapResult(), "Failed to pass approach object global plan to the controller.");
                            done = true;
                            return false;
                        }

                        else
                        {
                            ROS_INFO_STREAM("Publishing plan for goal");
                            publishPlan(plan, index);

                            //Change local controller parameters for object pushing
                            double velocity = 0.4;
                            double sim_time = 1.8;
                            //updateControllerParams(velocity, sim_time);

                            move_states_ = CONTROL;


                        }
                    }   

                    else 
                    {
                        as_->setAborted(sem_nav_msgs::TapResult(), "Failed to generate approach object global plans.");
                        resetState();

                        done = true; // Go out of while loop
                        
                        return false;
                    }
                }
                break;

                case CONTROL:
                {
                    ROS_INFO_STREAM("APPROACHGOAL: CONTROL");
                    if ( !(goal_monitor_->goalMonitor(reach_goal, tolerance)) )
                    {
                        //move_states_ = PLAN;
                        tc_->sendVelocityCommands();
                    }
                    else 
                    {
                        tc_->stop();
                        done = true;
                        return true;
                    }  

                    

                }
                break;
            }
        }

        return false;



    }

    void TapAction::resetState()
    {
        tap_action_states_ = APPROACHOBJECT;
        move_states_ = PLAN;          
    }

    
    void TapAction::computeGoals(const geometry_msgs::PoseStamped& reach_goal, sem_nav_msgs::MoveObjectGoals& current_goals)
	{
        std::vector<double> distances;
        unsigned int mx, my;

        for (int i = 0; i < object.semantics.semantic_positions.position.size(); i++)
        {

            distances.push_back( distanceCalculator(object.semantics.semantic_positions.position[i].x, 
            object.semantics.semantic_positions.position[i].y, robot_pose.geometric_pose.pose.position.x, robot_pose.geometric_pose.pose.position.y) );


        }

        std::vector<double>::iterator min_distance = std::min_element(distances.begin(), distances.end());

        int sp_number = std::distance(distances.begin(), min_distance); 

        // Fill goal for approach object
        current_goals.approach_object.header.frame_id = "/map";
        current_goals.approach_object.pose.position.x = object.semantics.semantic_positions.position[sp_number].x;
        current_goals.approach_object.pose.position.y = object.semantics.semantic_positions.position[sp_number].y;
        

        // Find goal for tapping the object
        geometry_msgs::PoseStamped object_pose;
        object_pose.header.frame_id = "/map";
        object_pose.pose = object.geometry.pose;
        findMoveObjectGoal(current_goals, object_pose);

        // Fill the received final goal for robot 
        current_goals.reach_goal = reach_goal;
        

        publishGoals(current_goals);

	    
	}

	void TapAction::findMoveObjectGoal(sem_nav_msgs::MoveObjectGoals& current_goals, const geometry_msgs::PoseStamped object_pose)
	{
        double x_rob = robot_pose.geometric_pose.pose.position.x;
        double y_rob = robot_pose.geometric_pose.pose.position.y;

        double x_obj = object_pose.pose.position.x;
        double y_obj = object_pose.pose.position.y;

        current_goals.move_object.header.frame_id = "/map";
        current_goals.move_object.pose.position.x = object_pose.pose.position.x;
        current_goals.move_object.pose.position.y = object_pose.pose.position.y;		

	}

    void TapAction::localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose)
	{
        robot_pose = *pose;		
	}

	void TapAction::publishGoals(sem_nav_msgs::MoveObjectGoals& current_goals) 
	{
        sem_nav_msgs::MoveObjectGoals goals = current_goals;

        geometry_msgs::Point p;
        int index = 0;
          
        goal_marker->addPoint(current_goals.approach_object, index);

        index = 1;
        goal_marker->addPoint(goals.move_object, index);

        index = 2;
        goal_marker->addPoint(goals.reach_goal, index);

        goals_pub.publish(goal_marker->marker_array);	
	   

	}

	void TapAction::publishPlan(nav_msgs::Path& plan, int index)
	{
        
        path_marker->addPathPoints(plan, index);

        paths_pub.publish(path_marker->marker_array);
		
	}

    void TapAction::clearPublish()
    {
        goal_marker->deleteAll();
        path_marker->deleteAll();
        paths_pub.publish(path_marker->marker_array);
        goals_pub.publish(goal_marker->marker_array);
    } 

    void TapAction::updateControllerParams(double velocity, double sim_time)
    {
        ROS_INFO_STREAM("UPDATE");
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double_param.name = "sim_time";
        double_param.value = sim_time;
        conf.doubles.push_back(double_param);

        double_param.name = "max_vel_x";
        double_param.value = velocity;
        conf.doubles.push_back(double_param);

        srv_req.config = conf;

        ros::service::call("/tap_action_node/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);

    }

    bool TapAction::planService(sem_nav_msgs::GetPlanObject::Request &req, sem_nav_msgs::GetPlanObject::Response &resp)
    {
        if(as_->isActive())
        {
            ROS_ERROR("tap_action must be in an inactive state to make a plan for an external user");
            return false;
        }

        //make sure we have a costmap for our planner
        if(global_costmap_ == NULL)
        {
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        object.instance.name = req.object.instance.name;
        semantic_map_query_->getObjectDynamic(object);
        
        

        sem_nav_msgs::MoveObjectGoals current_goals;        
        computeGoals(req.goal, current_goals);
        resp.goals = current_goals;
        //ROS_INFO_STREAM(current_goals);

        std::vector<geometry_msgs::PoseStamped> approach_object_plan, tap_object_plan, approach_goal_plan;
        approach_object_plan.clear();
        tap_object_plan.clear();

        planner_->makePlanObjectApproach(robot_pose.geometric_pose, current_goals.approach_object, approach_object_plan, object);
        ROS_INFO_STREAM(approach_object_plan.size());
        
        planner_->makePlanObjectPush(current_goals.approach_object, current_goals.move_object, tap_object_plan, object);
        ROS_INFO_STREAM(tap_object_plan.size());

        planner_->makePlanObjectApproach(current_goals.move_object, current_goals.reach_goal, approach_goal_plan, object);
        ROS_INFO_STREAM(approach_goal_plan.size()); 

    //    if ( ! (planner_->makePlanObjectApproach(robot_pose.geometric_pose, current_goals.approach_object, approach_object_plan, object) || planner_->makePlanObjectPush(current_goals.approach_object, current_goals.move_object, push_object_plan, object)
    //        || planner_->makePlanObjectApproach(current_goals.move_object, current_goals.reach_goal, approach_goal_plan, object)) )
    //    {
    //        ROS_ERROR("push action, Failed to make a plan");
    //        return false;
    //    }

        //copy all the plan into a message to send out
        resp.plan.poses.resize(approach_object_plan.size() + tap_object_plan.size() + approach_goal_plan.size());

        for(unsigned int i = 0; i < approach_object_plan.size(); ++i)
        {
            //resp.plan.poses[i] = approach_object_plan[i];
            resp.plan.poses.push_back(approach_object_plan[i]);
        }

        for(unsigned int i = 0; i < tap_object_plan.size(); ++i)
        {
            //resp.plan.poses[i] = push_object_plan[i];
            resp.plan.poses.push_back(tap_object_plan[i]);
        }

        for(unsigned int i = 0; i < approach_goal_plan.size(); ++i)
        {
            resp.plan.poses.push_back(approach_goal_plan[i]); 
        }

        return true;

    }
};


    