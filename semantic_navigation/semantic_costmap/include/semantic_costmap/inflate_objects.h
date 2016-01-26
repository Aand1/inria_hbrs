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

#ifndef INFLATE_OBJECTS_H_
#define INFLATE_OBJECTS_H_

#include <iostream>
#include <cmath>
#include <math.h> 
#include <list>
#include <vector>
#include <iostream>
#include <boost/thread.hpp>
#include <queue> 

// ROS related
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>  
#include <costmap_2d/layered_costmap.h>
#include <tf/transform_listener.h> 
#include <std_msgs/String.h>
#include <string> 
#include <semantic_map/Object.h> 
#include <semantic_map/semantic_map.h> 

namespace semantic_navigation_layers
{
	/**
	 * @class CellData
	 * @brief Storage for cell information used during obstacle inflation
	 */
	class CellData
	{
	public:
	  /**
	   * @brief  Constructor for a CellData objects
	   * @param  d The distance to the nearest obstacle, used for ordering in the priority queue
	   * @param  i The index of the cell in the cost map
	   * @param  x The x coordinate of the cell in the cost map
	   * @param  y The y coordinate of the cell in the cost map
	   * @param  sx The x coordinate of the closest obstacle cell in the costmap
	   * @param  sy The y coordinate of the closest obstacle cell in the costmap
	   * @return
	   */
	  CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
	      distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
	  {
	  }
	  double distance_;
	  unsigned int index_;
	  unsigned int x_, y_;
	  unsigned int src_x_, src_y_;
	};

	/**
	 * @brief Provide an ordering between CellData objects in the priority queue
	 * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b
	 */
	inline bool operator<(const CellData &a, const CellData &b)
	{
	  return a.distance_ > b.distance_;
	}


	class InflateObjects : public costmap_2d::CostmapLayer
	{
	public:
		InflateObjects(costmap_2d::Costmap2D* parent, ros::NodeHandle& nh, tf::TransformListener *tf, std::string object_type);
		virtual ~InflateObjects();

		virtual void initialize();

		virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  		virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  		//void matchSize();

  		/** @brief  Given a distance, compute a cost.
   		 * @param  distance The distance from an obstacle in cells
   		 * @return A cost value for the distance */
  		inline unsigned char computeCost(double distance) const;

  		/**
	     * @brief  Lookup pre-computed distances
	     * @param mx The x coordinate of the current cell
	     * @param my The y coordinate of the current cell
	     * @param src_x The x coordinate of the source cell
	     * @param src_y The y coordinate of the source cell
	     * @return
	     */
	    inline double distanceLookup(int mx, int my, int src_x, int src_y);

	    inline void enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
                                            unsigned int src_x, unsigned int src_y);

		unsigned int cellDistance(double world_dist);

	    void matchSize();

	    /**
		 * @brief Change the values of the inflation radius parameters
		 * @param inflation_radius The new inflation radius
		 * @param cost_scaling_factor The new weight
		 */
        void setInflationParameters(double inflation_radius, double cost_scaling_factor);


  	protected:
  		boost::shared_mutex* access_;

	private:
		tf::TransformListener* tf_;
        ros::NodeHandle nh_;    	
		std::string name_; 

		costmap_2d::Costmap2D* layer_grid_;
		semantic_map::SemanticMap* semantic_map_query;
		//std::list<ObjectBounds> object_list;
		std::list<semantic_map::Object> object_list;
		std::string object_type_;

		bool current_;

		double inflation_radius_, inscribed_radius_, weight_;

		bool* seen_;
		int seen_size_;
		unsigned int cell_inflation_radius_;

		std::priority_queue<CellData> inflation_queue_;

		double context_weight_;
      	double semantic_weight_;
      	double context_factor_;
      	double semantic_factor_;

	};

};


#endif 
