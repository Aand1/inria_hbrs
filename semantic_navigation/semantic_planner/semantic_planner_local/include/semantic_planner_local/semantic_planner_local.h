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

#ifndef _SEMANTIC_PLANNER_LOCAL_
#define _SEMANTIC_PLANNER_LOCAL_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h> 
#include <nav_core/base_local_planner.h>
#include <std_msgs/Bool.h>
#include <base_local_planner/trajectory_planner_ros.h>

#include <sem_nav_msgs/LocalPlannerConstraints.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


namespace semantic_planner 
{
    class SemanticPlannerLocal
    {
    public:
        /**
         * @brief  Default constructor for the SemanticPlannerGlobal object
         */
	    SemanticPlannerLocal();

	/**
         * @brief  Constructor for the SemanticPlannerGlobal object
         * @param  tf A reference to a TransformListener
         */
        SemanticPlannerLocal( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
        // SemanticPlannerLocal(tf::TransformListener* tf);

	/**
         * @brief  Default deconstructor for the SemanticPlannerGlobal object
         */
        ~SemanticPlannerLocal();

        /**
         * @brief  Initialization function for the SemanticPlannerGlobal object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
	//void initialize(std::string name, tf::TransformListener* tf, ros::NodeHandle &nh, costmap_2d::Costmap2DROS* costmap_ros);
         void initialize(tf::TransformListener* tf);

    bool setPlan(const std::vector< geometry_msgs::PoseStamped >& orig_global_plan)
    {
        return tp.setPlan(orig_global_plan);
    }

    bool sendVelocityCommands();

    void setParam(sem_nav_msgs::LocalPlannerConstraints& constraints);

    void stop();
    void back();

    protected:
        costmap_2d::Costmap2DROS* local_costmap_;
        tf::TransformListener* tf_;
        ros::NodeHandle nh_;
        bool initialized_;



    private:
        void planCb(const nav_msgs::Path::ConstPtr& plan);
        void triggerCb(const std_msgs::Bool::ConstPtr& trigger);

        base_local_planner::TrajectoryPlannerROS tp;

        //Publishers and subscribers
        ros::Publisher vel_pub_;
        ros::Subscriber trigger_sub_;
        ros::Subscriber plan_sub_;

        std::vector<geometry_msgs::PoseStamped>* controller_plan_;
        double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
        double controller_patience_;
        bool new_global_plan_;

        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

			
	};


}
#endif
