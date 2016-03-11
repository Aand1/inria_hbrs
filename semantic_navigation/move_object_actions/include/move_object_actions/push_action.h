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

#ifndef PUSH_ACTION_H_
#define PUSH_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sem_nav_msgs/PushAction.h>
#include <sem_nav_msgs/GetPlanObject.h> 
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <semantic_costmap/semantic_costmap_ros.h> 
#include <semantic_planner_global/semantic_planner_global.h>
#include <semantic_planner_local/semantic_planner_local.h>
#include <sem_nav_msgs/MoveObjectGoals.h>
#include <geometry_msgs/PoseStamped.h>
#include <semantic_map/Object.h>
#include <semantic_map/semantic_map.h> 

#include <goal_monitor/goal_monitor.h>
#include <sem_nav_msgs/SemanticPose.h>

#include <sem_nav_msgs/viz_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Path.h>  

#include <costmap_2d/costmap_2d_ros.h> 

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h> 


using namespace std;

namespace move_object
{
	//typedefs to help us out with the action server so that we don't hace to type so much
	typedef actionlib::SimpleActionServer<sem_nav_msgs::PushAction> PushActionServer;

    enum PushActionStates 
    {
        APPROACHOBJECT,
        PUSHOBJECT,
        APPROACHGOAL,
        ABORT,

    };

    enum MoveStates
    {
    	PLAN,
    	CONTROL,
    	REPLAN
    };

	
	class PushAction 
	{
	public:
		PushAction(tf::TransformListener& tf); 

        virtual ~PushAction();

	    /*
	    * @brief  Initialization function for the MoveRobot object
	    * @param  tf A reference to a TransformListener
	    * @param  planner_costmap A pointer to the global costmap to use for global planning
	    */
		void initialize();

        void executeCb(const sem_nav_msgs::PushGoalConstPtr& push_goal);

    protected:
        semantic_costmap::SemanticCostmapROS* global_costmap_; 
        costmap_2d::Costmap2DROS* local_costmap_;

    private:

  	    bool executeCycle(const sem_nav_msgs::MoveObjectGoals& goals);

  	    bool executeApproachObject(const geometry_msgs::PoseStamped& approach_object);

  	    //bool executePushObject(const geometry_msgs::PoseStamped& approach_object);
        bool executePushObject(const geometry_msgs::PoseStamped& move_object, const geometry_msgs::PoseStamped& start);

  	    bool executeApproachGoal(const geometry_msgs::PoseStamped& approach_object);
        bool executeApproachGoal(const geometry_msgs::PoseStamped& approach_object, const geometry_msgs::PoseStamped& start);

        void updateParamCb();

        void updateParams();

  	    void resetState();

        void computeGoals(const geometry_msgs::PoseStamped& reach_goal, sem_nav_msgs::MoveObjectGoals& current_goals);
        bool nearOtherObjects(double x, double y, semantic_map::Object& object_);
        void findMoveObjectGoal(sem_nav_msgs::MoveObjectGoals& current_goals, const geometry_msgs::PoseStamped object_pose);

        double distanceCalculator(double x1, double y1, double x2, double y2)
        {
            double x = x1 - x2; //calculating number to square in next step
            double y = y1 - y2;
            double dist;

            dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
            dist = sqrt(dist);                  

            return dist;
        }

        void localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose);

        void publishGoals(sem_nav_msgs::MoveObjectGoals& current_goals);

        void publishPlan(nav_msgs::Path& plan, int index);
        void clearPublish();

        void updateControllerParams(double velocity, double sim_time);

        /**
         * @brief  A service call that can be made when the action is inactive that will return a plan
         * @param  req The goal request
         * @param  resp The plan request
         * @return True if planning succeeded, false otherwise
         */
        bool planService(sem_nav_msgs::GetPlanObject::Request &req, sem_nav_msgs::GetPlanObject::Response &resp);

        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
    

	    PushActionServer* as_;
	    
	    //actionlib::SimpleActionServer<move_object_msgs::PushAction> as_;

	    //boost::shared_ptr<move_object::PushPlanner> planner_;
        tf::TransformListener& tf_;

    	/*
         * @brief To store copies of global and local costmaps
         */
        //costmap_2d::Costmap2DROS* planner_costmap_, *controller_costmap_;

        semantic_planner::SemanticPlannerGlobal* planner_; 
        semantic_planner::SemanticPlannerLocal* tc_;
        semantic_planner::GoalMonitor* goal_monitor_;

    	bool initialized_;

    	//ros::NodeHandle nh_;

        //Publishers
        //ros::Publisher action_goal_pub_;
        //Subscribers
        //ros::Subscriber goal_sub_;
      
        PushActionStates push_action_states_;
        MoveStates move_states_;

        //Parameters for global planner
        //geometry_msgs::PoseStamped planner_goal_;
        //global_planner::GlobalPlanner* planner_;

        //std::string robot_base_frame_, global_frame_;
        //double planner_frequency_, planner_patience_;

        std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        std::vector<geometry_msgs::PoseStamped>* latest_plan_;

        sem_nav_msgs::MoveObjectGoals* goals;

        sem_nav_msgs::SemanticPose robot_pose;

        //Subscribers
        ros::Subscriber semantic_localization;

        //Instance of SemanticMap class to query the semantic map
        semantic_map::SemanticMap* semantic_map_query_; 
        semantic_map::Object object;

        // Markers for vizualization
        viz_msgs::VisualizationMarker *goal_marker, *path_marker;

        // ROS publishers
        ros::Publisher paths_pub, goals_pub;
        ros::Publisher action_goal_pub_;
        ros::Subscriber goal_sub_; 

        ros::ServiceServer make_plan_srv_;



};





}
#endif
