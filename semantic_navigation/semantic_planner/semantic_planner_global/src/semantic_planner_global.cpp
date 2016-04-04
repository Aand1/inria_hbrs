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

 #include "semantic_planner_global/semantic_planner_global.h"

 using namespace semantic_planner;
using namespace global_planner;

 SemanticPlannerGlobal::SemanticPlannerGlobal(std::string name, semantic_costmap::SemanticCostmapROS* costmap) :
    global_costmap_(NULL), initialize_(false)

{
	initialization(name, costmap);
}

SemanticPlannerGlobal::~SemanticPlannerGlobal()
{
	

}

void SemanticPlannerGlobal::initialization(std::string name, semantic_costmap::SemanticCostmapROS* costmap) 
{
	if (!initialize_) 
	{
		ros::NodeHandle private_nh("~");

		global_costmap_ = costmap;

		//initialize ROS Global Planner
		initialize(name, global_costmap_);

		unsigned int cx = global_costmap_->getCostmap()->getSizeInCellsX(), cy = global_costmap_->getCostmap()->getSizeInCellsY();

	/*	global_planner::SemanticDijkstraExpansion* de = new global_planner::SemanticDijkstraExpansion(global_costmap_->getCostmap(), p_calc_, cx, cy);   
        semantic_planner_ = de;

        semantic_planner_->setLethalCost(253.0);
   		semantic_planner_->setNeutralCost(50.0);
    	semantic_planner_->setFactor(3.0); 
    	semantic_planner_->setHasUnknown(true);*/

    	//advertise a service for getting a plan
        make_plan_robot = private_nh.advertiseService("make_plan_robot", &SemanticPlannerGlobal::planServiceForMoveRobot, this);
        
        make_plan_object = private_nh.advertiseService("make_plan_object", &SemanticPlannerGlobal::planServiceForMoveObject, this);

        initialize_ = true;


	}

}

bool SemanticPlannerGlobal::planServiceForMoveRobot(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{
	geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
    {
        tf::Stamped<tf::Pose> global_pose;
        if(!global_costmap_->getRobotPose(global_pose))
        {
            ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
            return false;
        } 
      
        tf::poseStampedTFToMsg(global_pose, start);
    }
    else
    {
      start = req.start;
    }

    goal = req.goal;

    std::vector<geometry_msgs::PoseStamped> plan;     
    if(!makePlan(start, goal, plan) || plan.empty())
    {
        return false;
    } 

    //copy the plan into a message to send out
    resp.plan.poses.resize(plan.size());
    for(unsigned int i = 0; i < plan.size(); ++i)
    {
      resp.plan.poses[i] = plan[i];
    }

    return true;

}

bool SemanticPlannerGlobal::planServiceForMoveObject(sem_nav_msgs::GetPlanObject::Request &req, sem_nav_msgs::GetPlanObject::Response &resp)
{
	geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;
    semantic_map::Object object;

    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
    {
        tf::Stamped<tf::Pose> global_pose;
        if(!global_costmap_->getRobotPose(global_pose))
        {
            ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
            return false;
        } 
      
        tf::poseStampedTFToMsg(global_pose, start);
    }
    else
    {
      start = req.start;
      object = req.object;
    }

    goal = req.goal;

    std::vector<geometry_msgs::PoseStamped> plan;     
    if(!makePlanObject(start, goal, plan, object) || plan.empty())
    {
        return false;
    } 

    //copy the plan into a message to send out
    resp.plan.poses.resize(plan.size());
    for(unsigned int i = 0; i < plan.size(); ++i)
    {
      resp.plan.poses[i] = plan[i];
    }

    return true;

}

bool SemanticPlannerGlobal::makePlanObjectApproach(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                           std::vector<geometry_msgs::PoseStamped>& plan, semantic_map::Object& object) 
{

	if (!initialized_) 
	{
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) 
    {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) 
    {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!global_costmap_->getCostmap()->worldToMap(wx, wy, start_x_i, start_y_i)) 
    {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    
    worldToMap(wx, wy, start_x, start_y);
    

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!global_costmap_->getCostmap()->worldToMap(wx, wy, goal_x_i, goal_y_i)) 
    {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    
    worldToMap(wx, wy, goal_x, goal_y);
    

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = global_costmap_->getCostmap()->getSizeInCellsX(), ny = global_costmap_->getCostmap()->getSizeInCellsY();


    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
   
    splanner_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(global_costmap_->getCostmap()->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    unsigned int mx_, my_; // associated map coordinates of the costmap index under consideration.
    unsigned int omx_, omy_; // associated map coordinates of the object bounding box descibed in world coordinates.
    double owx_, owy_;


    bool found_legal = splanner_->computePotentials(global_costmap_->getCostmap()->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                   nx * ny * 2, potential_array_, object);


    //publishPotential(potential_array_);

    if (found_legal) 
    {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } 
        else 
        {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }
    else
    {
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    //orientation_filter_->processPath(start, plan);
    
    //publish the plan for visualization purposes
    //publishPlan(plan);
    delete potential_array_;

    return !plan.empty();

    


}	

bool SemanticPlannerGlobal::makePlanObjectPush(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                           std::vector<geometry_msgs::PoseStamped>& plan, semantic_map::Object& object) 
{

	geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    std::vector<geometry_msgs::PoseStamped> plan_one_, plan_two_;

    start_ = start;
    goal_.header.frame_id = "/map";
    goal_.pose = object.geometry.pose;

    makePlanObject(start_, goal_, plan_one_, object);

    start_.header.frame_id = "/map";
    start_.pose = object.geometry.pose;
    goal_ = goal;
    makePlanObject(start_, goal_, plan_two_, object);

    //plan.resize(plan_one_.size() + plan_two_.size());

    plan = plan_one_;

    plan.insert(plan.end(), plan_two_.begin(), plan_two_.end());

    //std::merge(plan_one_.begin(), plan_one_.end(), plan_two_.begin(), plan_two_.end(), plan.begin());

    return true;

}

bool SemanticPlannerGlobal::makePlanObject(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                           std::vector<geometry_msgs::PoseStamped>& plan, semantic_map::Object& object) {
    //boost::mutex::scoped_lock lock(mutex_);


    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!global_costmap_->getCostmap()->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    
    worldToMap(wx, wy, start_x, start_y);
    

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!global_costmap_->getCostmap()->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    
    worldToMap(wx, wy, goal_x, goal_y);
    

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = global_costmap_->getCostmap()->getSizeInCellsX(), ny = global_costmap_->getCostmap()->getSizeInCellsY();


    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    splanner_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(global_costmap_->getCostmap()->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    unsigned int mx_, my_; // associated map coordinates of the costmap index under consideration.
    unsigned int omx_, omy_; // associated map coordinates of the object bounding box descibed in world coordinates.
    double owx_, owy_;


    bool found_legal = splanner_->computePotentials(global_costmap_->getCostmap()->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_, object);

    planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);


    //publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    //orientation_filter_->processPath(start, plan);
    
    //publish the plan for visualization purposes
    //publishPlan(plan);
    delete potential_array_;

    return !plan.empty();
}        

/*bool SemanticPlannerGlobal::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                           std::vector<geometry_msgs::PoseStamped>& plan) 
{

}*/
