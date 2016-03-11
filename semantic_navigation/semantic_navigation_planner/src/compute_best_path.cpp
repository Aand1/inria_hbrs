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
#include <semantic_navigation_planner/compute_best_path.h>

namespace semantic_navigation_planner
{

    ComputeBestPath::ComputeBestPath(tf::TransformListener& tf, ros::NodeHandle &nh) : tf_(tf), nh_(nh)
    {
    	initialize();
    }


    void ComputeBestPath::initialize()
    {

    	//planner_ = new semantic_planner::SemanticPlannerGlobal( &tf_ );

    	semantic_map_query_ = new semantic_map::SemanticMap(nh_);

    	// initialize service query for make plan
        
        std::string move_robot_service = "/move_robot_action/make_plan";
        std::string push_action_service = "/push_action/make_plan";
        std::string tap_action_service = "/tap_action/make_plan";

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

        paths_pub = nh_.advertise<visualization_msgs::MarkerArray>("move_object_paths", 1);
        goals_pub = nh_.advertise<visualization_msgs::MarkerArray>("move_object_goals", 1);



        while(!ros::service::waitForService(move_robot_service, ros::Duration(3.0))) 
        {
            ROS_ERROR("Service %s not available - waiting.", move_robot_service.c_str());
        }
        move_robot_client = nh_.serviceClient<nav_msgs::GetPlan>(move_robot_service, true);

        while(!ros::service::waitForService(push_action_service, ros::Duration(3.0))) 
        {
            ROS_ERROR("Service %s not available - waiting.", push_action_service.c_str());
        }
        push_action_client = nh_.serviceClient<sem_nav_msgs::GetPlanObject>(push_action_service, true);

        while(!ros::service::waitForService(tap_action_service, ros::Duration(3.0))) 
        {
            ROS_ERROR("Service %s not available - waiting.", tap_action_service.c_str());
        }
        tap_action_client = nh_.serviceClient<sem_nav_msgs::GetPlanObject>(tap_action_service, true);

    }

    sem_nav_msgs::BestPath ComputeBestPath::computeBestPath(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal)
    {
        clearPublish();

        sem_nav_msgs::BestPath best_path;

        std::list<semantic_map::Object> object_list;
        semantic_map_query_->getObjectsDynamic(object_list, "LightObject");
        object_list.remove_if (is_not_light_object()); 
        
        sem_nav_msgs::GetPlanObject push_action_srv;
        sem_nav_msgs::GetPlanObject tap_action_srv;
        nav_msgs::GetPlan move_robot_srv;
        nav_msgs::Path path_plan;
        std::list<sem_nav_msgs::BestPath> paths;

        std::list<semantic_map::Object>::iterator objects_it = object_list.begin();
        //std::list<sem_nav_msgs::BestPath>::iterator costs_it = costs_list.begin();

        for (objects_it = object_list.begin(); objects_it != object_list.end(); objects_it++)
        {
            semantic_map::Object& object = *objects_it;

            if (object.semantics.affordance.compare("Push")==0)
            {
                push_action_srv.request.start = start;
                push_action_srv.request.goal = goal;
                push_action_srv.request.object = object;

                if(push_action_client.call(push_action_srv))
                {
                    
                    path_plan = push_action_srv.response.plan;

                    sem_nav_msgs::BestPath path;

                    path.cost.data = getPathCost(path_plan);

                    path.category = object.semantics.sub_category;
                    path.instance = object.instance.name;
                    paths.push_back(path);
                    //ROS_INFO_STREAM(path_plan.poses.size());

                    int index =0;
                    //publishGoalsObject(push_action_srv.response.goals);
                    publishPlan(path_plan, index);
                }
            }

            else if (object.semantics.affordance.compare("Tap")==0)
            {
                tap_action_srv.request.start = start;
                tap_action_srv.request.goal = goal;
                tap_action_srv.request.object = object;

                if(tap_action_client.call(tap_action_srv))
                {
                    
                    path_plan = tap_action_srv.response.plan;

                    sem_nav_msgs::BestPath path;

                    path.cost.data = getPathCost(path_plan);

                    path.category = object.semantics.sub_category;
                    path.instance = object.instance.name;
                    paths.push_back(path);
                    //ROS_INFO_STREAM(path_plan.poses.size());

                    int index =0;
                    //publishGoalsObject(push_action_srv.response.goals);
                    publishPlan(path_plan, index);
                }
            }
        }

        if(move_robot_client.call(move_robot_srv))
        {
                    
            path_plan = move_robot_srv.response.plan;

            sem_nav_msgs::BestPath path;

            path.cost.data = getPathCost(path_plan);

            path.category = "robot";
            paths.push_back(path);
                    //ROS_INFO_STREAM(path_plan.poses.size());

            int index = 2;
                    //publishGoalsObject(push_action_srv.response.goals);
            publishPlan(path_plan, index);
        }

        best_path.cost.data = 1000;

        std::list<sem_nav_msgs::BestPath>::iterator bp_it = paths.begin();
        //std::list<sem_nav_msgs::BestPath>::iterator costs_it = costs_list.begin();

        for (bp_it = paths.begin(); bp_it != paths.end(); bp_it++)
        {
            sem_nav_msgs::BestPath& best_path_temp = *bp_it;

	    ROS_INFO_STREAM(best_path_temp);

            if (best_path_temp.cost.data < best_path.cost.data && best_path_temp.cost.data != 0)
            {
                best_path = best_path_temp;
            }

        }
        ROS_INFO_STREAM("---------------");
        ROS_INFO_STREAM(best_path);

        /*    else if (object.semantics.affordance.compare("Tap")==0)
            {
                push_action_srv.request.start = start;
                push_action_srv.request.goal = goal;
                push_action_srv.request.object = object;

                if(push_action_client.call(push_action_srv))
                {
                    
                    path_plan = push_action_srv.response.plan;

                    sem_nav_msgs::BestPath path;

                    path.cost.data = getPathCost(path_plan);

                    path.entity = object.semantics.sub_category;
                    paths.push_back(path);
                    ROS_INFO_STREAM(path_plan.poses.size());

                    int index =0;
                    //publishGoalsObject(push_action_srv.response.goals);
                    publishPlan(path_plan, index);
                }
            }*/

        

        ros::Duration(2).sleep();
        clearPublish();

/*        std_msgs::String goal_object;       

        std::list<double> cost_list;
        cost_list.resize(object_list.size());

        std::list<nav_msgs::Path> path_list;

        sem_nav_msgs::GetPlanObject push_action_srv;
        nav_msgs::GetPlan move_robot_srv;
        nav_msgs::Path path_plan;

        std::list<semantic_map::Object>::iterator objects_it = object_list.begin();
        std::list<double>::iterator costs_it = cost_list.begin();

        for (objects_it = object_list.begin(); objects_it != object_list.end(); ++costs_it, objects_it++)
        {
            semantic_map::Object& object = *objects_it;
            double& cost = *costs_it;

            if (object.semantics.sub_category.compare("Box")==0)
            {

                push_action_srv.request.start = start;
                push_action_srv.request.goal = goal;
                push_action_srv.request.object = object;

                if(push_action_client.call(push_action_srv))
                {
                    ROS_INFO_STREAM("HERE");
                    path_plan = push_action_srv.response.plan;

                    path_list.push_back(path_plan);

                    cost = getPathCost(path_plan);
                    ROS_INFO_STREAM(cost);

                    int index =0;
                    publishPlan(path_plan, index);
                }

            }

        }*/

        return best_path;     

    }

    double ComputeBestPath::getPathCost(nav_msgs::Path& path_plan) 
    {
        double d;
        double path_length = 0;
        double rot_length = 0;

        double best_path = 1000;

        if ( path_plan.poses.empty() )
            return 0;

            
        geometry_msgs::PoseStamped last_pose = path_plan.poses[0];

        BOOST_FOREACH( const geometry_msgs::PoseStamped & p, path_plan.poses )
        {
            d = hypot(last_pose.pose.position.x - p.pose.position.x,
                      last_pose.pose.position.y - p.pose.position.y);
            
            path_length += d;

            last_pose = p;
        }
        
        return path_length;

    }

    void ComputeBestPath::publishPlan(nav_msgs::Path& plan, int index)
    {
        
        path_marker->addPathPoints(plan, index);

        paths_pub.publish(path_marker->marker_array);
        
    }

    void ComputeBestPath::publishGoalsObject(sem_nav_msgs::MoveObjectGoals goals)
    {
        geometry_msgs::Point p;
        int index = 2;
          
        goal_marker->addPoint(goals.approach_object, index);

        goal_marker->addPoint(goals.move_object, index);

        goal_marker->addPoint(goals.reach_goal, index);

        goals_pub.publish(goal_marker->marker_array);   
       

    }

    void ComputeBestPath::clearPublish()
    {
        goal_marker->deleteAll();
        path_marker->deleteAll();
        paths_pub.publish(path_marker->marker_array);
        goals_pub.publish(goal_marker->marker_array);
    } 


/*    double ComputeBestPath::getObjectPlanCost(const geometry_msgs::PoseStamped robot_pose, const geometry_msgs::PoseStamped robot_goal)
    {

    	object_list = semantic_map_query_->getSemanticMap();
    	object_list.remove_if (is_not_light_object());
    	object_paths_list.resize(object_list.size());


    	computeGoals(robot_pose, robot_goal);

    	computePaths(robot_pose);

    	double path_cost = getPathCostForObject();

    	//ROS_INFO_STREAM(path_cost);

    }

    void ComputeBestPath::callPlanningForRobot(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, nav_msgs::Path& path)
    {
        // Encode incoming message types into a GetPlan service type
        nav_msgs::GetPlan getPlanRequest;
        getPlanRequest.request.start = start;
        getPlanRequest.request.goal = goal;

        if(getPlan.call(getPlanRequest))
        {
            path = getPlanRequest.response.plan;
        }
        else
        {
            ROS_ERROR("Failed to call path planning service for robot");
        }

    }

    void ComputeBestPath::callPlanningForObject(const geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped& goal, 
                        nav_msgs::Path& path, semantic_map::Object& object)
    {
        // Encode incoming message types into a GetPlan service type
        sem_nav_msgs::GetPlanObject getPlanRequest;
        getPlanRequest.request.start = start;
        getPlanRequest.request.goal = goal;
        getPlanRequest.request.object = object;

        if(getPlanObject.call(getPlanRequest))
        {
            path = getPlanRequest.response.plan;
        }
        else
        {
            ROS_ERROR("Failed to call path planning service for object");
        }

    }

    void ComputeBestPath::computeGoals(const geometry_msgs::PoseStamped robot_pose, const geometry_msgs::PoseStamped robot_goal)
    {

        std::list<semantic_map::Object>::iterator obs_it;
        for (obs_it = object_list.begin(); obs_it != object_list.end(); ++obs_it)
        {
        	semantic_map::Object& object = *obs_it;

        	sem_nav_msgs::MoveObjectGoals object_goals;

        	std::vector<double> distances;

        	for (int i = 0; i < object.semantics.semantic_positions.position.size(); i++)
            {
                distances.push_back( distanceCalculator(object.semantics.semantic_positions.position[i].x, 
                    object.semantics.semantic_positions.position[i].y, robot_pose.pose.position.x, robot_pose.pose.position.y) );
            }

            std::vector<double>::iterator min_distance = std::min_element(distances.begin(), distances.end());

            int sp_number = std::distance(distances.begin(), min_distance); 

            // Fill goal for approach object
            object_goals.approach_object.header.frame_id = "/map";
            object_goals.approach_object.pose.position.x = object.semantics.semantic_positions.position[sp_number].x;
            object_goals.approach_object.pose.position.y = object.semantics.semantic_positions.position[sp_number].y;

            // Find goal for pushing the object
            geometry_msgs::PoseStamped object_pose;
            object_pose.header.frame_id = "/map";
            object_pose.pose = object.geometry.pose;
            findMoveObjectGoal(object_goals, robot_pose, object_pose);

            // Fill the received final goal for robot 
            object_goals.reach_goal = robot_goal; 

            object_goals_list.push_back(object_goals);

            ROS_INFO_STREAM(object_goals);

        }

        publishGoals();
    }

    void ComputeBestPath::computePaths(const geometry_msgs::PoseStamped robot_pose)
    {

    	std::list<sem_nav_msgs::MoveObjectGoals>::iterator goals_it = object_goals_list.begin();
        std::list<sem_nav_msgs::MoveObjectPaths>::iterator paths_it = object_paths_list.begin();
        std::list<semantic_map::Object>::iterator objects_it = object_list.begin();

        std::vector<geometry_msgs::PoseStamped> plan;

        for (objects_it = object_list.begin(); objects_it != object_list.end(); ++goals_it, ++paths_it, objects_it++)
        {
        	sem_nav_msgs::MoveObjectGoals& object_goals = *goals_it;
            sem_nav_msgs::MoveObjectPaths& object_paths = *paths_it;
            semantic_map::Object& object = *objects_it;

            // Find path from robot position to object(approach object path)
            planner_->makePlanObjectApproach(robot_pose, object_goals.approach_object, plan, object);
            //copy the plan into a MoveObjectPaths message
    	    object_paths.approach_object.poses.resize(plan.size());
    	    for(unsigned int i = 0; i < plan.size(); ++i)
    	    {
    	      	object_paths.approach_object.poses[i] = plan[i];

    	    }
    	    plan.clear();

    	    // Find path from object position to object reposition(move object path)
    	    planner_->makePlanObjectPush(object_goals.approach_object, object_goals.move_object, plan, object);
    	    //copy the plan into a MoveObjectPaths message
    	    object_paths.move_object.poses.resize(plan.size());
    	    for(unsigned int i = 0; i < plan.size(); ++i)
    	    {
    	      	object_paths.move_object.poses[i] = plan[i];
    	    }
    	    plan.clear();

    	    // Find path from object position to object reposition(move object path)
    	    planner_->makePlan(object_goals.move_object, object_goals.reach_goal, plan, object);
    	    //copy the plan into a MoveObjectPaths message
    	    object_paths.reach_goal.poses.resize(plan.size());
    	    for(unsigned int i = 0; i < plan.size(); ++i)
    	    {
    	      	object_paths.reach_goal.poses[i] = plan[i];
    	    }
    	    plan.clear();          

        }

        publishPlans();

    }

    void ComputeBestPath::findMoveObjectGoal(sem_nav_msgs::MoveObjectGoals& object_goals, const geometry_msgs::PoseStamped robot_pose, 
    	const geometry_msgs::PoseStamped object_pose)
    {
    	double x_rob = robot_pose.pose.position.x;
        double y_rob = robot_pose.pose.position.y;

        double x_obj = object_pose.pose.position.x;
        double y_obj = object_pose.pose.position.y;

        if ( x_obj >= x_rob ) 
        {
            object_goals.move_object.header.frame_id = "/map";
            object_goals.move_object.pose.position.x = object_pose.pose.position.x + 1.0;
        }
        else if ( x_obj < x_rob ) 
        {
            object_goals.move_object.header.frame_id = "/map";
            object_goals.move_object.pose.position.x = object_pose.pose.position.x - 1.0;
        }
        if ( y_obj >= y_rob )
        {
            object_goals.move_object.pose.position.y = object_pose.pose.position.y + 1.0;
        }
        else if ( y_obj < y_rob)
        {
            object_goals.move_object.pose.position.y = object_pose.pose.position.y - 1.0;
        }

    }

    void ComputeBestPath::publishGoals() 
    {
    	
    	for (std::list<sem_nav_msgs::MoveObjectGoals>::iterator obs_it = object_goals_list.begin(); obs_it != object_goals_list.end(); ++obs_it)
        {
        	sem_nav_msgs::MoveObjectGoals& goals = *obs_it;

        	geometry_msgs::Point p;
        	int index = 0;
          
            goal_marker->addPoint(goals.approach_object, index);

            index = 1;
            goal_marker->addPoint(goals.move_object, index);

            index = 2;
            goal_marker->addPoint(goals.reach_goal, index);

        }

        goals_pub.publish(goal_marker->marker_array);

    }

    void ComputeBestPath::publishPlans()
    {
    	std::list<sem_nav_msgs::MoveObjectPaths>::iterator paths_it; 

        for (paths_it = object_paths_list.begin(); paths_it != object_paths_list.end(); ++paths_it)
        {
            sem_nav_msgs::MoveObjectPaths& object_paths = *paths_it;

            int index = 0;
            path_marker->addPathPoints(object_paths.approach_object, index);

            index = 1;
            path_marker->addPathPoints(object_paths.move_object, index);

            index = 2;
            path_marker->addPathPoints(object_paths.reach_goal, index);

        }

        paths_pub.publish(path_marker->marker_array);

    }

    double ComputeBestPath::getPathCostForObject() 
    {
    	double d;
    	double path_length = 0;
        double rot_length = 0;

        double best_path = 1000;


             
        std::list<sem_nav_msgs::MoveObjectPaths>::iterator paths_it; 

        for (paths_it = object_paths_list.begin(); paths_it != object_paths_list.end(); paths_it++)
        {

            sem_nav_msgs::MoveObjectPaths& object_paths = *paths_it;
            path_length = 0;

            //join(join(object_paths.approach_object.poses, object_paths.move_object.poses), total_path.poses);

            //ROS_INFO_STREAM(final_path.poses.size());

            //ROS_INFO_STREAM(join(object_paths.approach_object.poses, object_paths.move_object.poses));

            if ( object_paths.approach_object.poses.empty()
                 && object_paths.move_object.poses.empty()
                 && object_paths.reach_goal.poses.empty() )
                return 0;

            

            geometry_msgs::PoseStamped last_pose = object_paths.approach_object.poses[0];

            BOOST_FOREACH( const geometry_msgs::PoseStamped & p, object_paths.approach_object.poses )
            {
                d = hypot(last_pose.pose.position.x - p.pose.position.x,
                                 last_pose.pose.position.y - p.pose.position.y);
                path_length += d;

                last_pose = p;
            }
            
            last_pose = object_paths.move_object.poses[0];
            BOOST_FOREACH( const geometry_msgs::PoseStamped & p, object_paths.move_object.poses )
            {
                d = hypot(last_pose.pose.position.x - p.pose.position.x,
                                 last_pose.pose.position.y - p.pose.position.y);

                path_length += d;

                last_pose = p;
            }

            last_pose = object_paths.reach_goal.poses[0];
            BOOST_FOREACH( const geometry_msgs::PoseStamped & p, object_paths.reach_goal.poses )
            {
                d = hypot(last_pose.pose.position.x - p.pose.position.x,
                                 last_pose.pose.position.y - p.pose.position.y);

                path_length += d;

                last_pose = p;
            }

            //ROS_INFO_STREAM(path_length);

            if (path_length <= best_path)
            {
            	best_path = path_length;
            }

        }

        return best_path;

    }*/

    

}




