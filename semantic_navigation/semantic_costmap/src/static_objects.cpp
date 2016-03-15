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
#include <semantic_costmap/static_objects.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace semantic_navigation_layers
{
	StaticObjects::StaticObjects(costmap_2d::Costmap2D* parent, ros::NodeHandle& nh, tf::TransformListener *tf, std::string global_frame, bool rolling_window, std::string object_type) :
		nh_(nh), tf_(tf), object_type_(object_type), size_updated(false)
	{
		
		initialize(parent, global_frame, rolling_window);

	}

	StaticObjects::~StaticObjects()
	{
		
	}

	void StaticObjects::initialize(costmap_2d::Costmap2D* parent, std::string global_frame, bool rolling_window)
	{


		layer_grid_ = parent;

		global_frame_ = global_frame;

		rolling_window_ = rolling_window;

		ros::NodeHandle g_nh;

		current_ = true;

		semantic_map_query = new semantic_map::SemanticMap(nh_);
		semantic_map_query->getObjectsStatic(object_list_, object_type_);
		//ROS_INFO_STREAM(object_list_.size());
		

		enabled_ = true;


		// Initialize parameters
		std::string map_topic;
  		nh_.param("map_topic", map_topic, std::string("map"));
  		nh_.param("first_map_only", first_map_only_, true);

  		nh_.param("track_unknown_space", track_unknown_space_, true);
  		nh_.param("use_maximum", use_maximum_, false);

  		int temp_lethal_threshold, temp_unknown_cost_value;
  		nh_.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  		nh_.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  		nh_.param("trinary_costmap", trinary_costmap_, true);

       lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
       unknown_cost_value_ = temp_unknown_cost_value;
       
    /*   std::string map_topic = "map";
       //subscribe_to_updates_ = false;
	   track_unknown_space_ = true;
	   use_maximum_ = false;
	   int temp_lethal_threshold, temp_unknown_cost_value;
	   temp_lethal_threshold = int(100);
	   temp_unknown_cost_value = int(-1);
	   trinary_costmap_ = true;
	   unknown_cost_value_ = temp_unknown_cost_value;
	   

	   lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
	   unknown_cost_value_ = temp_unknown_cost_value;*/

 
       // Subscribing to map topic
       if (map_sub_.getTopic() != ros::names::resolve(map_topic))
	   {
		    // we'll subscribe to the latched topic that the map server uses
		    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticObjects::incomingMap, this);
		    map_received_ = false;
		    has_updated_data_ = false;

		    ros::Rate r(10);

		    while (!map_received_ && g_nh.ok())
		    {
		      ros::spinOnce();
		      r.sleep();
		    }		 
		}

	}

	void StaticObjects::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
	{	
		//ROS_INFO_STREAM(object_type);

		
        //matchSize();
		unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
		double x = new_map->info.origin.position.x;
		double y = new_map->info.origin.position.y;
		
		resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
		layer_grid_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y);
        // resize costmap if size, resolution or origin do not match
        /*if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  		{
    		// only update the size of the costmap stored locally in this layer
    		resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  		}*/

		// only update the size of the costmap stored locally in this layer
		
  		unsigned int index = 0;

  		// initialize the costmap with static data
  		for (unsigned int i = 0; i < size_y; ++i)
  		{
    		for (unsigned int j = 0; j < size_x; ++j)
    		{
      			unsigned char value = new_map->data[index];
      	
   				costmap_[index] = interpretValue(value, j ,i);
   				
      			
      			++index;
    		}
  		}
  		map_frame_ = new_map->header.frame_id;

  		// we have a new map, update full size of map
  		x_ = y_ = 0;
  		width_ = size_x_;
  		height_ = size_y_;
  		map_received_ = true;
  		has_updated_data_ = true;

	  	// shutdown the map subscrber if firt_map_only_ flag is on
	  	if (first_map_only_)
	  	{
	    	ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
	    	map_sub_.shutdown();
	  	}



	}

	void StaticObjects::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
	{
		
		useExtraBounds(min_x, min_y, max_x, max_y);

		double wx, wy;

		mapToWorld(x_, y_, wx, wy);
		*min_x = std::min(wx, *min_x);
		*min_y = std::min(wy, *min_y);

		mapToWorld(x_ + width_, y_ + height_, wx, wy);
		*max_x = std::max(wx, *max_x);
		*max_y = std::max(wy, *max_y);

		has_updated_data_ = false;

	}

	void StaticObjects::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) 
  	{
  		updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  	/*	unsigned char* master_array = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

  		for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = j * span + min_i;
            for (int i = min_i; i < max_i; i++)
            {

                if (costmap_[it] != costmap_2d::NO_INFORMATION)
                {
                    master_array[it] = costmap_[it];
                    ROS_INFO_STREAM("LETHAL");
                }
                it++;

            }
        }*/
	   
  	}

  	void StaticObjects::matchSize()
	{
        resizeMap(layer_grid_->getSizeInCellsX(), layer_grid_->getSizeInCellsY(), layer_grid_->getResolution(),
            layer_grid_->getOriginX(), layer_grid_->getOriginY());

        size_updated = true;

	}

	bool StaticObjects::inObjectBoundingBox(unsigned int x, unsigned int y)
	{ 
	  	unsigned int mx, my;
	    inside_ = false;
	    
	    std::list<semantic_map::Object>::iterator obs_it;
        for (obs_it = object_list_.begin(); obs_it != object_list_.end(); ++obs_it)
        {
        	semantic_map::Object& object = *obs_it;

		    for (l_ = 0; l_ < 4; l_++) 
		    {
		    	double wx = object.geometry.bounding_box.vertices[l_].x;
		        double wy = object.geometry.bounding_box.vertices[l_].y;

		        layer_grid_->worldToMap(wx, wy, mx, my);

		        polyX[l_] = mx;
		        polyY[l_] = my;

		        //ROS_INFO_STREAM(mx);
		        //ROS_INFO_STREAM(my);
		        //ROS_INFO_STREAM("----");
		    }
	    
	        oddNodes_= false;
	    
	        j_ = 3;
	    
		    for (i_=0; i_<4; i_++) 
		    {
			    if ((polyY[i_]< y && polyY[j_]>=y || polyY[j_]< y && polyY[i_]>=y)
			       &&  (polyX[i_]<=x || polyX[j_]<=x)) 
			    {
			        if (polyX[i_]+(y-polyY[i_])/(polyY[j_]-polyY[i_])*(polyX[j_]-polyX[i_])<x) 
			        {
			          oddNodes_ = !oddNodes_; 
			        }
			    }
			    j_ = i_; 
		    }
	   
		    if (oddNodes_ == true)
		    {
		       inside_ = true;
		    }

	    }
	    return inside_;

	    return false;
	}




};
