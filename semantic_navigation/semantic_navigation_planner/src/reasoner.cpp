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

#include <semantic_navigation_planner/reasoner.h>
#include <sem_nav_msgs/SemanticCostmapConstraints.h> 


namespace semantic_navigation_planner
{
	Reasoner::Reasoner() 
	{
		ros::NodeHandle nh;
		sem_cm_pub_ = nh.advertise<sem_nav_msgs::SemanticCostmapConstraints>( "semantic_costmap_constraints", 0, true);
		
	}

	Reasoner::~Reasoner()
	{
		
	}

	void Reasoner::computeConstraints(std_msgs::String& context, sem_nav_msgs::Constraints& constraints)
	{
	//	sem_nav_msgs::Constraints constraints;
	//	sem_nav_msgs::SemanticCostmapConstraints sem_cm_const;
	//	sem_nav_msgs::LayerConstraints layer_constraints;

	if (context.data.compare("reach") == 0)
	{
		ROS_INFO_STREAM("REACH");
		constraints.local_planner_constraints.doubles.resize(2);
		constraints.local_planner_constraints.doubles[0].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[0].value = 0.4;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[1].value = 3.2;


		constraints.semantic_costmap_constraints.layer_constraints.resize(3);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.4;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
		constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.3;
		constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 20.0;
		

	}

	if (context.data.compare("manipulate") == 0)
	{
		ROS_INFO_STREAM("MANIPULATE");

		constraints.local_planner_constraints.doubles.resize(9);
		constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[0].value = 2.0;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
		constraints.local_planner_constraints.doubles[1].value = 2.0;
		constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
		constraints.local_planner_constraints.doubles[2].value = 2.0;

		constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[3].value = 0.25;
		constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
		constraints.local_planner_constraints.doubles[4].value = 0.0;
		constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
		constraints.local_planner_constraints.doubles[5].value = 0.27;
		constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
		constraints.local_planner_constraints.doubles[6].value = -0.27;
		constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
		constraints.local_planner_constraints.doubles[7].value = 0.3;

		constraints.local_planner_constraints.doubles[8].name = "sim_time";
		constraints.local_planner_constraints.doubles[8].value = 3.2;

		////////////////////////////////////////////////////////////////////////////////////////

		constraints.semantic_costmap_constraints.layer_constraints.resize(3);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.4;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
		constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.3;
		constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 20.0;
		

	}

	if (context.data.compare("carry") == 0)
	{
    	ROS_INFO_STREAM("CARRY");

		constraints.local_planner_constraints.doubles.resize(9);
		constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[0].value = 1.2;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
		constraints.local_planner_constraints.doubles[1].value = 1.2;
		constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
		constraints.local_planner_constraints.doubles[2].value = 1.2;

		constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[3].value = 0.12;
		constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
		constraints.local_planner_constraints.doubles[4].value = 0.0;
		constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
		constraints.local_planner_constraints.doubles[5].value = 0.3;
		constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
		constraints.local_planner_constraints.doubles[6].value = -0.3;
		constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
		constraints.local_planner_constraints.doubles[7].value = 0.35;

		constraints.local_planner_constraints.doubles[8].name = "sim_time";
		constraints.local_planner_constraints.doubles[8].value = 8.5;


		constraints.semantic_costmap_constraints.layer_constraints.resize(3);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.55;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 1.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.55;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 1.0;

		constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
		constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 1.0;
		

	}

	if (context.data.compare("emergency") == 0)
	{
		ROS_INFO_STREAM("EMERGENCY");
		constraints.local_planner_constraints.doubles.resize(9);
		constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[0].value = 2.8;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
		constraints.local_planner_constraints.doubles[1].value = 2.8;
		constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
		constraints.local_planner_constraints.doubles[2].value = 2.8;

		constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[3].value = 0.6;
		constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
		constraints.local_planner_constraints.doubles[4].value = 0.0;
		constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
		constraints.local_planner_constraints.doubles[5].value = 0.4;
		constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
		constraints.local_planner_constraints.doubles[6].value = -0.4;
		constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
		constraints.local_planner_constraints.doubles[7].value = 0.35;

		constraints.local_planner_constraints.doubles[8].name = "sim_time";
		constraints.local_planner_constraints.doubles[8].value = 1.4;


		constraints.semantic_costmap_constraints.layer_constraints.resize(2);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 1.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;
		

	}	

	else
	{

	}

	/*	if (input.data == 1)
		{
			// Update constraints for costmap layers
			layer_constraints.name = "heavy_objects_layer";
			layer_constraints.inscribed_radius = 1.0;
			layer_constraints.inflation_radius = 1.0;
			layer_constraints.context_factor = 1.0;

			//constraints.semantic_costmap_constraints.layer_constraints.push_back(layer_constraints);
            sem_cm_const.layer_constraints.push_back(layer_constraints);
			

			ROS_INFO_STREAM(sem_cm_const);

		}

		if (input.data == 2)
		{

		}

		if (input.data == 3)
		{

		}

		else
		{

		}

		sem_cm_pub_.publish(sem_cm_const);*/
		
	}

	void Reasoner::computeConstraintsBasedOnLocation(std_msgs::String& region, sem_nav_msgs::Constraints& constraints)
	{

		if (region.data.compare(0, 10, "livingroom") == 0)
		{
			ROS_INFO_STREAM("LIVINGROOM");
			constraints.local_planner_constraints.doubles.resize(9);
			constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
			constraints.local_planner_constraints.doubles[0].value = 2.0;
			constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
			constraints.local_planner_constraints.doubles[1].value = 2.0;
			constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
			constraints.local_planner_constraints.doubles[2].value = 2.0;

			constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
			constraints.local_planner_constraints.doubles[3].value = 0.3;
			constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
			constraints.local_planner_constraints.doubles[4].value = 0.0;
			constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
			constraints.local_planner_constraints.doubles[5].value = 0.3;
			constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
			constraints.local_planner_constraints.doubles[6].value = -0.3;
			constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
			constraints.local_planner_constraints.doubles[7].value = 0.4;

			constraints.local_planner_constraints.doubles[8].name = "sim_time";
			constraints.local_planner_constraints.doubles[8].value = 3.0;

			//////////////////////////////////////////////////
			constraints.semantic_costmap_constraints.layer_constraints.resize(3);
			constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
			constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.6;
			constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 1.0;

			constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
			constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.4;
			constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;

			constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
			constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.25;
			constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 20.0;
			

		}	

	if (region.data.compare(0, 7, "doorway") == 0)
	{
		ROS_INFO_STREAM("DOORWAY");
        constraints.local_planner_constraints.doubles.resize(9);
		constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[0].value = 1.2;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
		constraints.local_planner_constraints.doubles[1].value = 1.2;
		constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
		constraints.local_planner_constraints.doubles[2].value = 1.2;

		constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[3].value = 0.12;
		constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
		constraints.local_planner_constraints.doubles[4].value = 0.0;
		constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
		constraints.local_planner_constraints.doubles[5].value = 0.25;
		constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
		constraints.local_planner_constraints.doubles[6].value = -0.25;
		constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
		constraints.local_planner_constraints.doubles[7].value = 0.35;

		constraints.local_planner_constraints.doubles[8].name = "sim_time";
		constraints.local_planner_constraints.doubles[8].value = 8.0;

		///////////////////////////////////////////////////////////////////////////////////////////

		constraints.semantic_costmap_constraints.layer_constraints.resize(3);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.5;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.4;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
		constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.25;
		constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 10.0;
		

	}	

	if (region.data.compare(0, 8,"corridor") == 0) 
	{
		ROS_INFO_STREAM("CORRIDOR");
		constraints.local_planner_constraints.doubles.resize(9);
		constraints.local_planner_constraints.doubles[0].name = "acc_lim_x";
		constraints.local_planner_constraints.doubles[0].value = 2.3;
		constraints.local_planner_constraints.doubles[1].name = "acc_lim_y";
		constraints.local_planner_constraints.doubles[1].value = 2.3;
		constraints.local_planner_constraints.doubles[2].name = "acc_lim_theta";
		constraints.local_planner_constraints.doubles[2].value = 2.3;

		constraints.local_planner_constraints.doubles[3].name = "max_vel_x";
		constraints.local_planner_constraints.doubles[3].value = 0.5;
		constraints.local_planner_constraints.doubles[4].name = "min_vel_x";
		constraints.local_planner_constraints.doubles[4].value = 0.0;
		constraints.local_planner_constraints.doubles[5].name = "max_vel_theta";
		constraints.local_planner_constraints.doubles[5].value = 0.3;
		constraints.local_planner_constraints.doubles[6].name = "min_vel_theta";
		constraints.local_planner_constraints.doubles[6].value = -0.5;
		constraints.local_planner_constraints.doubles[7].name = "min_in_place_vel_theta";
		constraints.local_planner_constraints.doubles[7].value = 0.35;

		constraints.local_planner_constraints.doubles[8].name = "sim_time";
		constraints.local_planner_constraints.doubles[8].value = 2.2;

		//////////////////////////////////////////////////
		constraints.semantic_costmap_constraints.layer_constraints.resize(3);
		constraints.semantic_costmap_constraints.layer_constraints[0].name = "structural_objects";
		constraints.semantic_costmap_constraints.layer_constraints[0].inflation_radius = 0.7;
		constraints.semantic_costmap_constraints.layer_constraints[0].context_factor = 0.0;

		constraints.semantic_costmap_constraints.layer_constraints[1].name = "heavy_objects";
		constraints.semantic_costmap_constraints.layer_constraints[1].inflation_radius = 0.4;
		constraints.semantic_costmap_constraints.layer_constraints[1].context_factor = 10.0;

		constraints.semantic_costmap_constraints.layer_constraints[2].name = "light_objects";
		constraints.semantic_costmap_constraints.layer_constraints[2].inflation_radius = 0.25;
		constraints.semantic_costmap_constraints.layer_constraints[2].context_factor = 10.0;
		

	}	

	}


};
