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

#include <semantic_costmap/inflate_objects.h>

namespace semantic_navigation_layers
{
	InflateObjects::InflateObjects(costmap_2d::Costmap2D* parent, ros::NodeHandle& nh, tf::TransformListener *tf, std::string object_type)
		:  nh_(nh), tf_(tf), object_type_(object_type),
		   inscribed_radius_(0.26),
		   inflation_radius_(0.50),
		   weight_(100),
  		   cell_inflation_radius_(0),
		   seen_(NULL)
	{
		
		access_ = new boost::shared_mutex();
		costmap_ = NULL;

		layer_grid_ = parent;

		


	}
	
	InflateObjects::~InflateObjects()
	{
		
	}

	void InflateObjects::initialize()
	{
		boost::unique_lock < boost::shared_mutex > lock(*access_);

    	current_ = true;

    	if (seen_)
      		delete[] seen_;
        seen_ = NULL;
        seen_size_ = 0;

    	cell_inflation_radius_ = cellDistance(inflation_radius_);

    	semantic_map_query = new semantic_map::SemanticMap(nh_);
    	semantic_map_query->getObjectsStatic(object_list, object_type_);	

	}

	void InflateObjects::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y)
	{
		/*if (object_type_.compare("StructuralObject") == 0)
		{
			semantic_map_query->getObjectsStatic(object_list, object_type_);
		}
		else
		{
			semantic_map_query->getObjectsDynamic(object_list, object_type_);
		}*/

	    //semantic_map_query->getObjectsStatic(object_list, object_type_);
		
	}

	void InflateObjects::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
	{
		//cell_inflation_radius_ = cellDistance(inflation_radius_);

		inflation_queue_.empty();

		unsigned char* master_array = master_grid.getCharMap();
    	unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();


  		if (seen_ == NULL) 
  		{
		    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
		    seen_size_ = size_x * size_y;
		    seen_ = new bool[seen_size_];
  		}
		else if (seen_size_ != size_x * size_y)
		{
		    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
		    delete[] seen_;
		    seen_size_ = size_x * size_y;
		    seen_ = new bool[seen_size_];
		}
		memset(seen_, false, size_x * size_y * sizeof(bool));


		//////////////////////////////////////////////////////////////////////
		std::list<semantic_map::Object>::iterator obs_it;
        for (obs_it = object_list.begin(); obs_it != object_list.end(); ++obs_it)
        {
            semantic_map::Object& object = *obs_it;
            unsigned int min_x_ = size_x;// = 1000;
            unsigned int min_y_ = size_y;// = 1000;
            unsigned int max_x_ = 0;// = 0.0;
            unsigned int max_y_ = 0;// = 0.0;
            unsigned int mx, my;

            for ( int i = 0; i < object.geometry.bounding_box.vertices.size(); i++ )
            {
                double wx = object.geometry.bounding_box.vertices[i].x;
                double wy = object.geometry.bounding_box.vertices[i].y;

                layer_grid_->worldToMap(wx, wy, mx, my);

                min_x_ = std::min(min_x_, mx);
                min_y_ = std::min(min_y_, my);
                max_x_ = std::max(max_x_, mx);
                max_y_ = std::max(max_y_, my);
                //min_x_ = std::max(min_x_, mx);
  				//min_y_ = std::max(min_y_, my);
  				//max_x_ = std::min(max_x_, mx);
  				//max_y_ = std::min(max_y_, my);

            }

            for (int j = min_y_; j < max_y_; j++)
			{
			    for (int i = min_x_; i < max_x_; i++)
			    {
			      	int index = master_grid.getIndex(i, j);
			      	unsigned char cost = master_array[index];
			      	if (cost == costmap_2d::LETHAL_OBSTACLE)
			      	{
			        	enqueue(master_array, index, i, j, i, j);
			     	}
			    }
			}

        }

        while (!inflation_queue_.empty())
		{
		    // get the highest priority cell and pop it off the priority queue
		    const CellData& current_cell = inflation_queue_.top();

		    unsigned int index = current_cell.index_;
		    unsigned int mx = current_cell.x_;
		    unsigned int my = current_cell.y_;
		    unsigned int sx = current_cell.src_x_;
		    unsigned int sy = current_cell.src_y_;

		    // pop once we have our cell info
		    inflation_queue_.pop();

		    // attempt to put the neighbors of the current cell onto the queue
		    if (mx > 0)
		      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
		    if (my > 0)
		      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
		    if (mx < size_x - 1)
		      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
		    if (my < size_y - 1)
		      enqueue(master_array, index + size_x, mx, my + 1, sx, sy);
		}

        


		

		
	}

	void InflateObjects::matchSize()
	{
	  //boost::unique_lock < boost::shared_mutex > lock(*access_);
	/*  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
	  resolution_ = costmap->getResolution();
	  cell_inflation_radius_ = cellDistance(inflation_radius_);
	  

	  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
	  if (seen_)
	    delete seen_;
	  seen_size_ = size_x * size_y;
  	  seen_ = new bool[seen_size_];*/

	  resolution_ = layer_grid_->getResolution();
	  cell_inflation_radius_ = cellDistance(inflation_radius_);
	  

	  unsigned int size_x = layer_grid_->getSizeInCellsX(), size_y = layer_grid_->getSizeInCellsY();
	  if (seen_)
	    delete seen_;
	  seen_size_ = size_x * size_y;
  	  seen_ = new bool[seen_size_];

	}

	inline unsigned char InflateObjects::computeCost(double distance) const
  	{
    	unsigned char cost = 0;
    	if (distance == 0)
    		cost = costmap_2d::LETHAL_OBSTACLE;
	    else if (distance * resolution_ <= inscribed_radius_)
	      	cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
	    else
	    {   
		   	// make sure cost falls off by Euclidean distance
      	/*	double euclidean_distance = distance * resolution_;
      		double inflation_distance = euclidean_distance - inscribed_radius_;

      		double final_weight_ = context_factor_*context_weight_ + semantic_factor_*semantic_weight_;

      		double factor = exp(-1.0 * final_weight_ * inflation_distance);
      		
      		cost = (unsigned char)((254 - 1) * factor);*/


      		// make sure cost falls off by Euclidean distance
		    double euclidean_distance = distance * resolution_;
		    double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
		    cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
		}
    	return cost;
  	}

	inline double InflateObjects::distanceLookup(int mx, int my, int src_x, int src_y)
	{
	    unsigned int dx = abs(mx - src_x);
	    unsigned int dy = abs(my - src_y);
	    return hypot(dx, dy);
	}

	inline void InflateObjects::enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
                                            unsigned int src_x, unsigned int src_y)
	{
		  
	    if (!seen_[index])
	    {
		    //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
		    double distance = distanceLookup(mx, my, src_x, src_y);

		    //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
		    if (distance > cell_inflation_radius_)
		      return;

		    //assign the cost associated with the distance from an obstacle to the cell
		    unsigned char cost = computeCost(distance);
		    unsigned char old_cost = grid[index];

		    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
		    {
		      
		      grid[index] = cost;
		      //ROS_INFO_STREAM("First condition");
		    }
		    else
		    {
		      grid[index] = std::max(old_cost, cost);
		      
		    }
		    //push the cell data onto the queue and mark
		    seen_[index] = true;
		    CellData data(distance, index, mx, my, src_x, src_y);
		    inflation_queue_.push(data);
	    }
	}

	unsigned int InflateObjects::cellDistance(double world_dist)
	{
	   	return layer_grid_->cellDistance(world_dist);
	}

	void InflateObjects::setInflationParameters(double inflation_radius, double cost_scaling_factor)
	{
		
		if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
		{
		    inflation_radius_ = inflation_radius;
		    cell_inflation_radius_ = cellDistance(inflation_radius_);

		    weight_ = cost_scaling_factor;

		}

	}

	

};
