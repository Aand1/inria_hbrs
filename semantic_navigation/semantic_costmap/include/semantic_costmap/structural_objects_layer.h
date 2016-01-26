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

#ifndef STRUCTURAL_OBJECTS_LAYER_H_
#define STRUCTURAL_OBJECTS_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h> 
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>  

#include <semantic_costmap/static_objects.h>
#include <semantic_costmap/dynamic_objects.h>
#include <semantic_costmap/inflate_objects.h>

#include <dynamic_reconfigure/server.h> 
#include <semantic_costmap/StructuralObjectsPluginConfig.h>


namespace semantic_navigation_layers
{
	class StructuralObjectsLayer : public costmap_2d::CostmapLayer
    {
    public:
    	StructuralObjectsLayer();
    	~StructuralObjectsLayer();

    	virtual void onInitialize();
    	virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual void activate();
  	    virtual void deactivate();
  		virtual void reset();
        virtual void setParameters(const sem_nav_msgs::LayerConstraints& constraints);

        void matchSize();

    private:

        void reconfigureCB(semantic_costmap::StructuralObjectsPluginConfig &config, uint32_t level);

        costmap_2d::Costmap2D* local_costmap_;

        std::string object_type;

    	StaticObjects* so_;
    	InflateObjects* io_;

        dynamic_reconfigure::Server<semantic_costmap::StructuralObjectsPluginConfig> *dsrv_;
      
    };
}; 
#endif 
