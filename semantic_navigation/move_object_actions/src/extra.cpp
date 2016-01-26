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
	PushAction::PushAction(tf::TransformListener& tf, ros::NodeHandle &nh) :
        tf_(tf), nh_(nh),
        as_(NULL),
        planner_plan_(NULL), latest_plan_(NULL),
        //initialized_(false),
        //planner_costmap_(NULL),
        planner_(NULL), tc_(NULL), goal_monitor_(NULL),
        goals(NULL)

    {
    	//ros::NodeHandle private_nh("~");
        //ros::NodeHandle nh;

        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        ///controller_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        //controller_costmap_->start();

	    semantic_map_query_ = new semantic_map::SemanticMap(nh_);

        global_costmap_ = new semantic_costmap::SemanticCostmapROS("global_costmap", tf_);

        planner_ = new semantic_planner::SemanticPlannerGlobal( "global_planner" , global_costmap_);


	    //planner_ = new semantic_planner::SemanticPlannerGlobal( &tf_ );

        //tc_ = new semantic_planner::SemanticPlannerLocal( "push_action_lp", &tf_, nh_, controller_costmap_ );
        //tc_ = new semantic_planner::SemanticPlannerLocal( &tf_ );

        goal_monitor_ = new semantic_planner::GoalMonitor();
    	//initialize(tf, planner_costmap, controller_costmap);
    	
    	as_ = new PushActionServer(ros::NodeHandle(), "push_action", boost::bind(&PushAction::executeCb, this, _1), false);

        push_action_states_ = APPROACHOBJECT;

         
        //semantic_planner::SemanticPlannerGlobal global_planner( &tf_ );

        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        semantic_localization = nh.subscribe<sem_nav_msgs::SemanticPose>("semantic_localization", 1, boost::bind(&PushAction::localizationCb, this, _1));

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

	    paths_pub = nh_.advertise<visualization_msgs::MarkerArray>("move_object_paths", 10, true);
	    goals_pub = nh_.advertise<visualization_msgs::MarkerArray>("move_object_goals", 10, true);

        as_->start();

        goals = new sem_nav_msgs::MoveObjectGoals();


    }

    PushAction::~PushAction() 
	{
        if(as_ != NULL)
            delete as_;

        /*if(planner_costmap_ != NULL)
      		delete planner_costmap_;*/
        if(controller_costmap_ != NULL)
      		delete controller_costmap_;

      	delete planner_plan_;
        delete latest_plan_;
      
		
	}

	void PushAction::initialize(tf::TransformListener* tf, costmap_2d::Costmap2DROS* planner_costmap, costmap_2d::Costmap2DROS* controller_costmap)
    {
    /*    if(!initialized_)
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


            

        }*/
    }


    void PushAction::executeCb(const sem_nav_msgs::PushGoalConstPtr& push_goal)
    {

        ROS_INFO_STREAM("Received request for pushing an object");

        sem_nav_msgs::PushGoal new_goal = *push_goal;
        sem_nav_msgs::MoveObjectGoals current_goals;
        
        computeGoals(new_goal, current_goals);

        // initialize the states
        push_action_states_ = APPROACHOBJECT;

        ros::NodeHandle n;

        while(n.ok())
        {
            if(as_->isPreemptRequested())
            {
                if(as_->isNewGoalAvailable())
                {
                	ROS_INFO_STREAM("Received request for pushing an object");

                	sem_nav_msgs::PushGoal new_goal = *as_->acceptNewGoal();
                	computeGoals(new_goal, current_goals);
                    // initialise states
                    push_action_states_ = APPROACHOBJECT;
                                                                
                }
           
                else 
                {
                      //if we've been preempted explicitly we need to shut things down
                      resetState();

                      //notify the ActionServer that we've successfully preempted
                      ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                      as_->setPreempted();

                      //we'll actually return from execute after preempting
                      return;
                }
            }
      
            bool done = executeCycle(current_goals);   

            if (done)
            {
            	as_->setSucceeded(sem_nav_msgs::PushResult(), "Goal reached.");
                return;
            }

        }

        //if the node is killed then we'll abort and return
        as_->setAborted(sem_nav_msgs::PushResult(), "Aborting the action because the node has been killed");
        return;
    }

    bool PushAction::executeCycle(const sem_nav_msgs::MoveObjectGoals& goals)
    {
        
        switch(push_action_states_)
        {
            case APPROACHOBJECT:
                {
                    ROS_INFO_STREAM("APPROACHOBJECT");
                    bool approached_object = executeApproachObject(goals.approach_object);

                    if (approached_object)
                    {
                    	//as_->setSucceeded(sem_nav_msgs::PushResult(), "Goal reached.");
                        push_action_states_ = PUSHOBJECT;

                        
                    }

                    else 
                        return false;

                }
                break;


            case PUSHOBJECT:
                {
                    ROS_INFO_STREAM("PUSHOBJECT");
                    bool pushed_object = 1;//executePushObject(goals.move_object);

                    if (pushed_object)
                    {
                        
                        push_action_states_ = APPROACHGOAL;
                    }

                }    
                break;

            case APPROACHGOAL:
                {
                    ROS_INFO_STREAM("APPROACHGOAL");
                    bool approached_goal = 1;//executeApproachGoal(goals.reach_goal);

                    if (approached_goal)
                    {
                        //as_->setSucceeded(sem_nav_msgs::PushResult(), "Goal reached.");
                        return true;
                    }

                }    
                break;

        }

        return false;
    }

    bool PushAction::executeApproachObject(const geometry_msgs::PoseStamped& approach_goal)
    {
    	planner_plan_->clear();

        bool done = false;

        double tolerance = 0.6;

        semantic_map::Object object_;
        object_.instance.name = "corrogated_box1";
        semantic_map_query_->getObject(object_);
        
	    nav_msgs::Path plan;
	    
	    int index = 0;

        move_states_ = PLAN;
       
        while (!done)
        {
            switch(move_states_)
            {
                case PLAN:
                                        
                    //ROS_INFO_STREAM("APPROACHOBJECT: PLAN");
                    bool gotPlan;
                    gotPlan = planner_->makePlanObjectApproach(robot_pose.geometric_pose, approach_goal, *planner_plan_, object_);
                    

                    if (gotPlan)
                    {    
                    	//copy the plan into a MoveObjectPaths message
					    plan.poses.resize(planner_plan_->size());
					    for(unsigned int i = 0; i < planner_plan_->size(); ++i)
					    {
					      	plan.poses[i] = planner_plan_->at(i);
					    }
					    
                        if( 1==1/*!tc_->setPlan(*planner_plan_)*/)
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::PushResult(), "Failed to pass approach object global plan to the controller.");
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
                        as_->setAborted(sem_nav_msgs::PushResult(), "Failed to generate approach object global plans.");
                        resetState();
                        done = true;
                        return false;
                    }

                    break;
                /*{
                    ROS_INFO_STREAM("APPROACHOBJECT: PLAN");

                    planner_plan_->clear();

                    bool gotPlan = planner_->makePlanObject(start, goal, *planner_plan_, object);
                    ROS_INFO_STREAM(gotPlan);

                    if (gotPlan)
                    {    
                        if(!tc_->setPlan(*planner_plan_))
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::PushResult(), "Failed to pass approach object global plan to the controller.");
                            //return true;
                        }

                        else
                            move_states_ = CONTROL;
                    }      

                    else 
                    {
                        as_->setAborted(sem_nav_msgs::PushResult(), "Failed to pass approach object global plan to the controller.");
                        resetState();
                        return false;
                    }
                }
                    
                    //return false;                     
                break;*/

                case CONTROL:
                {
                /*    //ROS_INFO_STREAM("APPROACHOBJECT: CONTROL");
                    geometry_msgs::PoseStamped goal;

                    bool goal_reached;
                    goal_reached = goal_monitor_->goalMonitor(approach_goal, tolerance);

                    //ROS_INFO_STREAM(goal_reached);
                    
                    //goal_monitor_->goalReached();

                    if ( !(goal_monitor_->goalMonitor(approach_goal, tolerance)) )
                    {

                    	tc_->sendVelocityCommands();
                    }
                    else 
                    {
                    	tc_->stop();
                    	done = true;
                    }*/
                    return true;	

                }
                break;
            }
        }

        return true;

    }

    bool PushAction::executePushObject(const geometry_msgs::PoseStamped& move_object)
    {
        bool done = false;

        double tolerance = 0.5;

        semantic_map::Object object_;
        object_.instance.name = "corrogated_box1";
        semantic_map_query_->getObject(object_);
        
	    nav_msgs::Path plan;
	    geometry_msgs::PoseStamped start;
	    start.header.frame_id = "/map";
	    start.pose = object_.geometry.pose;
	    
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
                    gotPlan = planner_->makePlanObjectPush(start, move_object, *planner_plan_, object_);

                    if (gotPlan)
                    {   
                    	//copy the plan into a MoveObjectPaths message
					    plan.poses.resize(planner_plan_->size());
					    for(unsigned int i = 0; i < planner_plan_->size(); ++i)
					    {
					      	plan.poses[i] = planner_plan_->at(i);
					    }

                        if( 1==1 /*!tc_->setPlan(*planner_plan_)*/ )
                        {
                            //ABORT and SHUTDOWN COSTMAPS
                            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                            //resetState();

                            as_->setAborted(sem_nav_msgs::PushResult(), "Failed to pass move object global plan to the controller.");
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
                        as_->setAborted(sem_nav_msgs::PushResult(), "Failed to generate approach object global plans.");
                        resetState();
                        done = true;
                        return false;
                    }
                }

                    break;


                case CONTROL:
                {
                	semantic_map_query_->getObject(object_);

            		geometry_msgs::PoseStamped goal;
	    			goal.header.frame_id = "/map";
	    			goal.pose = object_.geometry.pose;

                    ROS_INFO_STREAM("PUSHOBJECT: Control");

                    if ( !(goal_monitor_->goalMonitor(move_object, tolerance)) )
                    {

                    	//tc_->sendVelocityCommands();
                    	
                    }
                    else 
                    {
                    	//tc_->stop();
                    	done = true;
                    }

                }
                break;
            }

        }

        return true;

    }

    bool PushAction::executeApproachGoal(const geometry_msgs::PoseStamped& reach_goal)
    {
        return true;

    }

    void PushAction::resetState()
    {
        push_action_states_ = APPROACHOBJECT;
        move_states_ = PLAN;  

    }

    
    void PushAction::computeGoals(const sem_nav_msgs::PushGoal& new_goal, sem_nav_msgs::MoveObjectGoals& current_goals)
	{

	    object.instance.name = new_goal.object_instance.name;
	    semantic_map_query_->getObject(object);

	    std::vector<double> distances;

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

	    // Find goal for pushing the object
	    geometry_msgs::PoseStamped object_pose;
	    object_pose.header.frame_id = "/map";
	    object_pose.pose = object.geometry.pose;
	    findMoveObjectGoal(current_goals, object_pose);

	    // Fill the received final goal for robot 
	    current_goals.reach_goal = new_goal.reach_goal;
	    

	    publishGoals(current_goals);
	}

	void PushAction::findMoveObjectGoal(sem_nav_msgs::MoveObjectGoals& current_goals, const geometry_msgs::PoseStamped object_pose)
	{
		double x_rob = robot_pose.geometric_pose.pose.position.x;
	    double y_rob = robot_pose.geometric_pose.pose.position.y;

	    double x_obj = object_pose.pose.position.x;
	    double y_obj = object_pose.pose.position.y;

	    if ( x_obj >= x_rob ) 
	    {
	        current_goals.move_object.header.frame_id = "/map";
	        current_goals.move_object.pose.position.x = object_pose.pose.position.x + 1.0;
	    }
	    else if ( x_obj < x_rob ) 
	    {
	        current_goals.move_object.header.frame_id = "/map";
	        current_goals.move_object.pose.position.x = object_pose.pose.position.x - 1.0;
	    }
	    if ( y_obj >= y_rob )
	    {
	        current_goals.move_object.pose.position.y = object_pose.pose.position.y + 1.0;
	    }
	    else if ( y_obj < y_rob)
	    {
	        current_goals.move_object.pose.position.y = object_pose.pose.position.y - 1.0;
	    }

	}

    void PushAction::localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose)
	{
		robot_pose = *pose;	
		//ROS_INFO_STREAM(robot_pose);	

	/*	robot_pose.geometric_pose.header.stamp = ros::Time::now();
	    robot_pose.geometric_pose.header.frame_id = "map";
	    robot_pose.geometric_pose.pose.position.x = 3.5;
	    robot_pose.geometric_pose.pose.position.y = -2.0;
	    robot_pose.geometric_pose.pose.position.z = 0.0;
	    robot_pose.geometric_pose.pose.orientation.x = 0.0;
	    robot_pose.geometric_pose.pose.orientation.y = 0.0;
	    robot_pose.geometric_pose.pose.orientation.z = 0.0;
	    robot_pose.geometric_pose.pose.orientation.w = 0.0;*/
	}

	void PushAction::publishGoals(sem_nav_msgs::MoveObjectGoals& current_goals) 
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

	void PushAction::publishPlan(nav_msgs::Path& plan, int index)
	{
		path_marker->addPathPoints(plan, index);

		paths_pub.publish(path_marker->marker_array);
	}
};

