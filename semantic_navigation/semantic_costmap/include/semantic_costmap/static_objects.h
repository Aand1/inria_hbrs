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

#ifndef STATIC_OBJECTS_H_
#define STATIC_OBJECTS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h> 
#include <nav_msgs/OccupancyGrid.h> 

#include <semantic_map/semantic_map.h> 

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace semantic_navigation_layers
{
	class StaticObjects : public costmap_2d::CostmapLayer
	{
	public:
		StaticObjects(costmap_2d::Costmap2D* parent, ros::NodeHandle& nh, tf::TransformListener *tf, std::string global_frame, bool rolling_window, std::string object_type);

		~StaticObjects();

		void initialize(costmap_2d::Costmap2D* parent, std::string global_frame, bool rolling_window);

		virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
     
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        void matchSize();


    private:
    	bool inObjectBoundingBox(unsigned int x, unsigned int y);

    	unsigned char interpretValue(unsigned char value, unsigned int x, unsigned int y)
		{
			  unsigned int x_ = x;
    		  unsigned int y_ = y;
			  // check if the static value is above the unknown or lethal thresholds
			  if (track_unknown_space_ && value == unknown_cost_value_)
			    return NO_INFORMATION;
			  else if (!track_unknown_space_ && value == unknown_cost_value_)
			    return FREE_SPACE;
			  else if (value >= lethal_threshold_  && inObjectBoundingBox(x_,y_) )
			    return LETHAL_OBSTACLE;
			  else if (trinary_costmap_)
			    return FREE_SPACE;

			  double scale = (double) value / lethal_threshold_;
			  return scale * LETHAL_OBSTACLE;
		}


        void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

        costmap_2d::Costmap2D* layer_grid_;
        semantic_map::SemanticMap* semantic_map_query;
        std::list<semantic_map::Object> object_list_;
        std::string object_type_;

        // Variables related to bounding box of objects
        float  polyX[4], polyY[4];
        bool  oddNodes_;
        bool inside_;
        int i_, j_, k_, l_;

        tf::TransformListener* tf_;
      	ros::NodeHandle nh_;

      	std::string global_frame_; ///< @brief The global frame for the costmap 
      	std::string map_frame_;  /// @brief frame that map is located in   
      	bool track_unknown_space_;
  		bool use_maximum_;
  		bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing	
  		bool trinary_costmap_;
  		unsigned char lethal_threshold_, unknown_cost_value_;

  		bool map_received_;
  		bool has_updated_data_;
  		unsigned int x_, y_, width_, height_;
  		bool size_updated;

  		bool rolling_window_;

  		//Subscribers
  		ros::Subscriber map_sub_;
      	

	};

};

#endif 
