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
 *                Dr. Anne Spalanzani (Inria)
 *********************************************************************/

#include <semantic_costmap/semantic_costmap_ros.h>
#include <string>
 


namespace semantic_costmap
{


    SemanticCostmapROS::SemanticCostmapROS(std::string name, tf::TransformListener& tf) :  Costmap2DROS(name, tf)

    {
        
        
    }

    SemanticCostmapROS::~SemanticCostmapROS()
    {
        
    }

    void SemanticCostmapROS::startCostmap()
    {
        std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = layered_costmap_->getPlugins();
        // check if we're stopped or just paused
        if (stopped_)
        {
            // if we're stopped we need to re-subscribe to topics
            for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
                ++plugin)
            {
              (*plugin)->activate();
              (*plugin)->reset();
            }
            stopped_ = false;
            stop_publisher_ = false;
        }
        stop_updates_ = false;
          

        // block until the costmap is re-initialized.. meaning one update cycle has run
        ros::Rate r(100.0);
        while (ros::ok() && !initialized_)
           r.sleep();

    }

    void SemanticCostmapROS::stopCostmap()
    {
        
        stop_updates_ = true;

        std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = layered_costmap_->getPlugins();

        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
            ++plugin)
        {
            (*plugin)->deactivate();

        }

        initialized_ = false;
        stopped_ = true; 

        getCostmap()->resetMaps();
    }

    void SemanticCostmapROS::setParameters(const sem_nav_msgs::SemanticCostmapConstraints& constraints)
    {
        stopCostmap();
        //ROS_INFO_STREAM("SemanticCostmapROS");
        std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = layered_costmap_->getPlugins();

        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
            ++plugin)
        {
            std::string plugin_name = (*plugin)->getName();


            
            for(int i = 0; i < constraints.layer_constraints.size(); i++)
            {
                //ROS_INFO_STREAM(plugin_name);
                //ROS_INFO_STREAM(name_ + "/" + constraints.layer_constraints[i].name);
                //ROS_INFO_STREAM("-----------");
                if (plugin_name.compare(name_ + "/" + constraints.layer_constraints[i].name) == 0)
                {
                    
                    (*plugin)->setParameters(constraints.layer_constraints[i]);
                    (*plugin)->activate();
                }

            }
            //(*plugin)->setParameters();

        }

    }


}