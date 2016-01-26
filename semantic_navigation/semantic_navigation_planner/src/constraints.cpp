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

#include <semantic_navigation_planner/constraints.h>
#include <sem_nav_msgs/SemanticCostmapConstraints.h> 


namespace semantic_navigation_planner
{
	Constraints::Constraints() 
	{
		ros::NodeHandle nh;
		sem_cm_pub_ = nh.advertise<sem_nav_msgs::SemanticCostmapConstraints>( "semantic_costmap_constraints", 0, true);
		
	}

	Constraints::~Constraints()
	{
		
	}

	void Constraints::computeConstraints(std_msgs::Int32& input)
	{
		sem_nav_msgs::Constraints constraints;
		sem_nav_msgs::SemanticCostmapConstraints sem_cm_const;
		sem_nav_msgs::LayerConstraints layer_constraints;

		if (input.data == 1)
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

		sem_cm_pub_.publish(sem_cm_const);
		
	}


};
