/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Niranjan Deshpande
*********************************************************************/

#include "semantic_localization/semantic_localization.h"

using namespace semantic_localization;

SemanticLocalization::SemanticLocalization()
{
	//tf_ = tf;
	//nh_ = nh;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
  
    private_nh.param<double>("publish_frequency", publish_frequency, 10);
    private_nh.param<std::string>("map_frame", map_frame, "map");
    private_nh.param<std::string>("base_frame", base_frame, "/base_link");
  
    semantic_localization_publisher = nh.advertise<sem_nav_msgs::SemanticPose>("semantic_localization", 1);

    semantic_map_sub_ = nh.subscribe("semantic_map_message", 1, &SemanticLocalization::semanticMapMessageCallback, this);

    tf_prefix = tf::getPrefixParam(private_nh);

    semantic_map_query_ = new semantic_map::SemanticMap(nh);

    
    semantic_map_query_->getRegions(region_list);

    //query_regions = new query_semantic_map::QueryRegions(nh);

    //querySemanticMap();
  
    
}

SemanticLocalization::~SemanticLocalization()
{
	
}

void SemanticLocalization::semanticMapMessageCallback(const semantic_map::SemanticMapMessage& semantic_map)
{
    ROS_INFO_STREAM("Semantic map message received");
}

void SemanticLocalization::executeCycle()
{
  
	bool tf_ok = true;
    try {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
    } catch(tf::TransformException ex) {
      //ROS_ERROR("-------> %s", ex.what());
      tf_ok = false;
    }
	
	if(tf_ok) 
	{
		robot_pose.geometric_pose = getGeometricPose();

		robot_pose.semantic_pose = getSemanticPose(robot_pose.geometric_pose);
    //ROS_INFO_STREAM("Publishing semantic robot localization");
		publishSemanticLocalization();
	}
    
    

}

semantic_map::RegionInstance SemanticLocalization::getSemanticPose(geometry_msgs::PoseStamped& geometric_pose)
{
	//std::string instance = livingroom-01;
  semantic_map::RegionInstance current_position;


  std::list<semantic_map::Region>::iterator regions_it = region_list.begin();
        //std::list<sem_nav_msgs::BestPath>::iterator costs_it = costs_list.begin();

  for (regions_it = region_list.begin(); regions_it != region_list.end(); regions_it++)
  {
    semantic_map::Region& region = *regions_it;

    if ( inObjectBoundingBox(region, geometric_pose.pose.position.x, geometric_pose.pose.position.y) )
    {
        
        current_position.name = region.instance.name;
    }


  }

	

	//instance.name = "livingroom-01";

	return current_position;

}

geometry_msgs::PoseStamped SemanticLocalization::getGeometricPose()
{

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "/"+map_frame;
      
      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();
      
      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      //pose.geometric_pose = pose_stamped;
      
      //semantic_localization_publisher.publish(pose);


    return pose_stamped;

}

void SemanticLocalization::publishSemanticLocalization()
{
	//ROS_INFO_STREAM(robot_pose.semantic_pose);
  semantic_localization_publisher.publish(robot_pose);
}

void SemanticLocalization::querySemanticMap()
{
    regions_list;
}


bool SemanticLocalization::inObjectBoundingBox(semantic_map::Region& region_, unsigned int x, unsigned int y)
{ 
    inside_ = false;
    
    
    for (l_ = 0; l_ < 4; l_++) 
    {
        polyX[l_] = region_.geometry.bounding_box.vertices[l_].x;
        polyY[l_] = region_.geometry.bounding_box.vertices[l_].y;
        
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

   
    return inside_;
}





