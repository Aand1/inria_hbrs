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

#include <sem_nav_msgs/viz_msgs.h>

namespace viz_msgs
{
	VisualizationMarker::VisualizationMarker(std::string& name, geometry_msgs::Vector3& scale, std_msgs::Int32& size) 
	    : name_(name),
	      scale_(scale), 
	      size_(size)
	{
		initialize();

	}

	VisualizationMarker::~VisualizationMarker()
	{

	}

	void VisualizationMarker::initialize()
	{
	/*	// Initialize visualization message for goal_marker 
	    goal_marker.header.frame_id = "/map";
	    goal_marker.header.stamp = ros::Time::now();
	    goal_marker.ns = namespace_;
	    goal_marker.id = 1;
	    goal_marker.type = visualization_msgs::Marker::POINTS;
	    goal_marker.action = visualization_msgs::Marker::ADD;
	    goal_marker.scale.x = 0.15;
	    goal_marker.scale.y = 0.15;
	    goal_marker.scale.z = 0.01;
	    goal_marker.color.r = 0.0f;
	    goal_marker.color.g = 0.0f;
	    goal_marker.color.b = 1.0f;
	    goal_marker.color.a = 1.0;
	    goal_marker.lifetime = ros::Duration(0);
	    goal_marker.colors.resize(size_.data);
	    goal_marker.points.resize(size_.data);*/

	    //Initializing visualization array message move_object_paths
    	marker_array.markers.resize(size_.data);
    	marker_array.markers.resize(size_.data);

	    for ( int i = 0; i < size_.data; i++)
	    {
	        marker_array.markers[i].header.frame_id = "map";
	        marker_array.markers[i].header.stamp = ros::Time();
		marker_array.markers[i].lifetime = ros::Duration();
	        marker_array.markers[i].ns = name_;    
	        marker_array.markers[i].type = visualization_msgs::Marker::POINTS;
	        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
	        //goal_markers.markers[i].scale.x = 0.15;
	        //goal_markers.markers[i].scale.y = 0.15;
	        //goal_markers.markers[i].scale.z = 0.01;
	        marker_array.markers[i].scale = scale_;
	        marker_array.markers[i].color.a = 1.0;
	        
	        if (i == 0)
	        {
	            marker_array.markers[i].color.r = 1.0f;
	            marker_array.markers[i].color.g = 0.0f;
	            marker_array.markers[i].color.b = 0.0f;
	            marker_array.markers[i].id = i;
	        }
	        else if (i == 1)
	        {
	            marker_array.markers[i].color.r = 0.0f;
	            marker_array.markers[i].color.g = 1.0f;
	            marker_array.markers[i].color.b = 0.0f;
	            marker_array.markers[i].id = i;

	        }
	        else
	        {
	            marker_array.markers[i].color.r = 0.0f;
	            marker_array.markers[i].color.g = 0.0f;
	            marker_array.markers[i].color.b = 1.0f;
	            marker_array.markers[i].id = i;
	        }
	    }


	}

	void VisualizationMarker::addPoint(const geometry_msgs::PoseStamped& point, int index)
	{
		//goal_marker.points.push_back(p);
        //goal_marker.colors.push_back(c);

        geometry_msgs::Point p;
        p.x = point.pose.position.x;
        p.y = point.pose.position.y; 
       // p1.x = 1.0;
        //p1.y = 0.0;
        marker_array.markers[index].action = visualization_msgs::Marker::ADD;

        marker_array.markers[index].points.push_back(p);
	}

	void VisualizationMarker::addPathPoints(const nav_msgs::Path& points, int index)
	{
		geometry_msgs::Point p;
		for (int i = 0; i < points.poses.size(); i++)
        {
        	p.x = points.poses[i].pose.position.x;
            p.y = points.poses[i].pose.position.y;  
             
            marker_array.markers[index].action = visualization_msgs::Marker::ADD;

            marker_array.markers[index].points.push_back(p);        

        }

	}

	void VisualizationMarker::deleteAll()
	{
		for (int i = 0; i < marker_array.markers.size(); i++)
		{
		
			marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

			for (int j = 0; j < marker_array.markers[i].points.size(); j++)
			{
			    marker_array.markers[i].points.pop_back();
			}
                        //marker_array.markers.empty();
		}
		
	}

}
