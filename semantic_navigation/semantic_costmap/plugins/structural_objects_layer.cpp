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

#include <semantic_costmap/structural_objects_layer.h>
#include <costmap_2d/costmap_math.h> 
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(semantic_navigation_layers::StructuralObjectsLayer, costmap_2d::Layer)


namespace semantic_navigation_layers
{
	StructuralObjectsLayer::StructuralObjectsLayer() : local_costmap_(NULL)
	{
		
	    costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
        //heavy_objects_costmap_.setDefaultValue(0);
        

	}

	StructuralObjectsLayer::~StructuralObjectsLayer()
	{
		
	}

	void StructuralObjectsLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;

        local_costmap_ = new costmap_2d::Costmap2D();

        local_costmap_->setDefaultValue(0);

        object_type = "StructuralObject";


        std::string global_frame_ = layered_costmap_->getGlobalFrameID();

        bool rolling_window_ = layered_costmap_->isRolling();

     	so_ = new StaticObjects(local_costmap_, nh, tf_, global_frame_, rolling_window_, object_type); 
        
        io_ = new InflateObjects(local_costmap_, nh, tf_, object_type);
     	//io_ = new InflateObjects(layered_costmap_, tf_, name_); 
     	io_->initialize();

        dynamic_reconfigure::Server<semantic_costmap::StructuralObjectsPluginConfig>::CallbackType cb = boost::bind(
        &StructuralObjectsLayer::reconfigureCB, this, _1, _2);

        dsrv_ = new dynamic_reconfigure::Server<semantic_costmap::StructuralObjectsPluginConfig>(ros::NodeHandle("~/" + name_));
        dsrv_->setCallback(cb);
    
 
    }

    void StructuralObjectsLayer::reconfigureCB(semantic_costmap::StructuralObjectsPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;

        io_->setInflationParameters(config.inflation_radius, config.cost_scaling_factor);
        
    }

    void StructuralObjectsLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
            return;

        if (!update_)
            return;
        //matchSize();

    	so_->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
        
    	io_->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);


    }

    void StructuralObjectsLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                           int max_j)
    {
    	if (!enabled_)
            return;

        if (!update_)
            return;
        //do_->updateCosts(heavy_objects_costmap_, min_i, min_j, max_i, max_j);
        so_->updateCosts(*local_costmap_, min_i, min_j, max_i, max_j);

    	io_->updateCosts(*local_costmap_, min_i, min_j, max_i, max_j);
        //io_->updateCosts(*local_costmap_, min_i, min_j, max_i, max_j);
        
    
        unsigned char* master_array = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        unsigned char* layer_array = local_costmap_->getCharMap();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = j * span + min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //unsigned char old_cost = master_array[it];
                //if ( old_cost < layer_array[it] )
                //{
                master_array[it] = layer_array[it];
                //}
                it++;

            }
        }


    
    }

    void StructuralObjectsLayer::matchSize()
  	{
	    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();

        local_costmap_->resizeMap(300.0, 350.0, 0.05, -7.50, -8.75);

	    //local_costmap_->resizeMap(360.0, 180.0, 0.05, -9.3, -3.78);


        //so_->matchSize();
        
        io_->matchSize();

  	}

    void StructuralObjectsLayer::activate()
    {
        enabled_ = true;
        ROS_INFO_STREAM("Started StructuralObjectsLayer");

    }

    void StructuralObjectsLayer::deactivate()
    {
        enabled_ = false;
        update_ = false;
        ROS_INFO_STREAM("Stopped StructuralObjectsLayer");
    }

    void StructuralObjectsLayer::reset()
    {
        update_ = true;
    }

    void StructuralObjectsLayer::setParameters(const sem_nav_msgs::LayerConstraints& constraints)
    {
        ROS_INFO_STREAM("Updating StructuralObjectsLayer CONSTRAINTS");
        local_costmap_->resizeMap(300.0, 350.0, 0.05, -7.50, -8.75);
        //local_costmap_->resizeMap(330.0, 160.0, 0.05, -8.375, -3.325);
        //local_costmap_->resizeMap(360.0, 180.0, 0.05, -9.3, -3.78); This is for robocup map
        update_ = true;
        io_->setInflationParameters(constraints.inflation_radius, constraints.context_factor);
    }


    
}
