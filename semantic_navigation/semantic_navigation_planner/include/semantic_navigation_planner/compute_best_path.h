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

#ifndef COMPUTE_BEST_PATH_H_
#define COMPUTE_BEST_PATH_H_ 

 
#include <cmath>
#include <math.h>
#include <typeinfo> 
#include <list>
#include <vector>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/range.hpp>
#include <boost/range/join.hpp>

// ROS related
#include <ros/ros.h>
//#include <semantic_planner_global/semantic_planner_global.h> 
#include <semantic_map/semantic_map.h>
#include <nav_msgs/GetPlan.h>
#include <sem_nav_msgs/GetPlanObject.h>
#include <semantic_map/Object.h> 
#include <sem_nav_msgs/MoveObjectGoals.h>
#include <sem_nav_msgs/MoveObjectPaths.h>
#include <sem_nav_msgs/SemanticPose.h>
#include <sem_nav_msgs/BestPath.h> 
#include <geometry_msgs/PoseStamped.h>  
#include <visualization_msgs/MarkerArray.h>
#include <sem_nav_msgs/viz_msgs.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_listener.h> 
#include <std_msgs/Bool.h>
#include <nav_msgs/GetPlan.h> 
//#define join boost::join 


// a struct to eliminate objects which do not belong to LightObjects category from the complete list of objects.
struct is_not_light_object 
{
	bool operator() (const semantic_map::Object& obj) 
	{ 
		return (obj.semantics.category != "LightObject"); 
	}
}; 

namespace semantic_navigation_planner
{
	class ComputeBestPath
	{
	public:
	    /**
	     * @brief Constructor for the class
	     * @param name The name of the class
	     * @param tf A reference to a TransformListener
	     */
	    ComputeBestPath(tf::TransformListener& tf, ros::NodeHandle &nh);

	    /**
	     * @brief  Destructor - Cleans up
	     */
	    ~ComputeBestPath(); 

	    /*
		 * @brief  Initialization function for the ComputeBestPath object
		 * @param  
		 * @param  
		 */
	    void initialize();

	    double getObjectPlanCost(const geometry_msgs::PoseStamped robot_pose, const geometry_msgs::PoseStamped robot_goal);
	    double getRobotPlanCost();

	    sem_nav_msgs::BestPath computeBestPath(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal);

	private:
		/**
		 * @brief Calls the semantic_gloabl_planner node for getting a path plan for robot
		 * @param start The state position for the planner
		 * @param goal  The goal position for the planner
		 * @param path  The path message to be filled with computed path.
		 */
		void callPlanningForRobot(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, 
						nav_msgs::Path& path);
	    /**
		 * @brief Calls the semantic_gloabl_planner node for getting a path plan for an object
		 * @param start The state position for the planner
		 * @param goal  The goal position for the planner
		 * @param path  The path message to be filled with computed path.
		 */
		void callPlanningForObject(const geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped& goal, 
						nav_msgs::Path& path, semantic_map::Object& object);

		/**
	     * @brief  Finds paths for all candidate objects. Paths for each object consist 
	     *         of a path to approach object, push the object and then a path to robot goal
	     *         from pushed location.
	     */
		void computeGoals(const geometry_msgs::PoseStamped robot_pose, const geometry_msgs::PoseStamped robot_goal);

		/**
	     * @brief  Finds paths for all candidate objects. Paths for each object consist 
	     *         of a path to approach object, push the object and then a path to robot goal
	     *         from pushed location.
	     */
		void computePaths(const geometry_msgs::PoseStamped robot_pose);

		/**
		 * @brief Calculates distance between two positions
		 * @param x1 The x cordinate of first position
		 * @param y1 The y cordinate of first position
		 * @param x2 The x cordinate of second position
		 * @param y2 The y cordinate of second position
		 * @return Distance between two positions
		 */
		double distanceCalculator(double x1, double y1, double x2, double y2)
	    {
			double x = x1 - x2; //calculating number to square in next step
			double y = y1 - y2;
			double dist;

			dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
			dist = sqrt(dist);                  

			return dist;
	    }

	    void findMoveObjectGoal(sem_nav_msgs::MoveObjectGoals& object_goals, const geometry_msgs::PoseStamped robot_pose, 
		const geometry_msgs::PoseStamped object_pose);
	

		void publishGoalsObject(sem_nav_msgs::MoveObjectGoals goals); 
		void publishPlans(); 
		void publishPlan(nav_msgs::Path& plan, int index);
		void clearPublish();

		double getPathCostForObject(); 

		double getPathCost(nav_msgs::Path& path_plan); 


		ros::NodeHandle nh_;
		tf::TransformListener& tf_;

		//semantic_planner::SemanticPlannerGlobal* planner_; 

		//Instance of SemanticMap class to query the semantic map
		semantic_map::SemanticMap* semantic_map_query_; 

		// Client to get a path from semantic_planner_global Service
	    ros::ServiceClient getPlan, getPlanObject;	

	    // Variables to store instances, paths and goals for pushing of objects in the vicinity of robot
	 	//std::list<semantic_map::Object> object_list;
	    //std::list<sem_nav_msgs::MoveObjectGoals> object_goals_list;
	    //std::list<sem_nav_msgs::MoveObjectPaths> object_paths_list;	

	    // ROS publishers
	    ros::Publisher paths_pub, goals_pub;

	    // Markers for vizualization
	    viz_msgs::VisualizationMarker *goal_marker, *path_marker;

	    // Services to get plans from different actions
	    ros::ServiceClient push_action_client;
	    ros::ServiceClient tap_action_client;
	    ros::ServiceClient move_robot_client;
	    
	};

};

#endif