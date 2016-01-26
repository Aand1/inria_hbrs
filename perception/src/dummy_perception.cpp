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

#include <perception/dummy_perception.h>
#include <math.h> 

using namespace gazebo;
using namespace physics;

#define PI 3.14159265

DummyPerception::DummyPerception(const ros::NodeHandle &nh) : nh_(nh)
{
	//sp.resize(4);

	access_ = new mutex_t();

	semantic_map = new semantic_map::SemanticMap(nh_);

	gazebo_ros_objects_state_subscriber = nh_.subscribe<perception::ObjectsState>("gazebo/objects_state", 1000, boost::bind(&DummyPerception::objectsStateCallback, this, _1));

	double map_update_frequency = 1;
    map_update_thread_shutdown_ = false;

    map_update_thread_ = new boost::thread(boost::bind(&DummyPerception::updateMapLoop, this, map_update_frequency));

}

DummyPerception::~DummyPerception()
{
	map_update_thread_shutdown_ = true;

	delete access_;

    if (map_update_thread_ != NULL)
    {
    	map_update_thread_->join();
    	delete map_update_thread_;
  	}
	
}

void DummyPerception::updateMap()
{

}

void DummyPerception::printUpdates(std::list<Object>& list)
{
	std::list<Object>::iterator obs_it;

  	for (obs_it = list.begin(); obs_it != list.end(); ++obs_it)
  	{
    	semantic_map::Object& object = *obs_it;
    	
    	ROS_INFO_STREAM(object);
  	}

}

void DummyPerception::updateMapLoop(double frequency)
{
	if (frequency == 0.0)
    	return;

    ros::NodeHandle nh;
    ros::Rate r(frequency);
  
    while (nh.ok() && !map_update_thread_shutdown_)
    {
    	//ROS_INFO_STREAM("Updating semantic map");
    	boost::unique_lock<mutex_t> lock(*access_);
    	computeValues();
    	//ROS_INFO_STREAM(object_list_.size());
    	lock.unlock();
    	    	
    	//printUpdates(object_list_);
    	semantic_map->updateMap(object_list_);
    	r.sleep();
    
    	// make sure to sleep for the remainder of our cycle time
    	if (r.cycleTime() > ros::Duration(1 / frequency))
    	{
      		ROS_WARN("Semantic Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
      	}
  
  	}
	
}

void DummyPerception::computeBbAndSp(const Vector2& center, const double w, const double h, double angle, semantic_map::Object& obj)
{
	// Compute and assign bounding box
/*	ROS_INFO_STREAM(cos(angle*PI/180.0));
    ROS_INFO_STREAM(w);
    ROS_INFO_STREAM(sin(angle*PI/180.0));
    ROS_INFO_STREAM(h);

    ROS_INFO_STREAM("========");*/
	Vector2 X( cos(angle*PI/180.0), sin(angle*PI/180.0));
	Vector2 Y( -sin(angle*PI/180.0), cos(angle*PI/180.0));

	//Vector2 X( cos(angle), sin(angle));
	//Vector2 Y( -sin(angle), cos(angle));

	// Adding tolerance for bounding box
	double w_ = w + 0.5;
	double h_ = h + 0.5;
	
	if ( angle < 45 || angle > 135)
	{
		X *= h_ / 2;
    	Y *= w_ / 2;
	}

	else
	{
		X *= w_ / 2;
    	Y *= h_ / 2;
	}


	//X *= h / 2;
    //Y *= w / 2;

/*    ROS_INFO_STREAM(X.x);
    ROS_INFO_STREAM(X.y);
    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM(Y.x);
    ROS_INFO_STREAM(Y.y);*/

   
    //ROS_INFO_STREAM("Angle1");
    corner[0] = center - X - Y;
    corner[1] = center + X - Y;
    corner[2] = center + X + Y;
    corner[3] = center - X + Y;
    
    
   

    
 
    for (int i = 0; i < 4; i++)
    {
    	

      	obj.geometry.bounding_box.vertices[i].x =  precision(corner[i].x); 
       	obj.geometry.bounding_box.vertices[i].y =  precision(corner[i].y);  
       	
    }


    // Compute and assign semantic positions
    sp[0].x =  precision(center.x);
    sp[1].y =  precision(center.y);
    sp[2].x =  precision(center.x);
    sp[3].y =  precision(center.y);

    sp[0].z =  precision(0.0);
    sp[1].z =  precision(0.0);
    sp[2].z =  precision(0.0);
    sp[3].z =  precision(0.0);

    if (obj.semantics.category.compare("HeavyObject") == 0)
    {
    	sp[0].y =  precision(center.y - (h / 2 + 0.3));
        sp[1].x =  precision(center.x + (w / 2 + 0.3));    
        sp[2].y =  precision(center.y + (h / 2 + 0.3));
        sp[3].x =  precision(center.x - (w / 2 + 0.3));
    }
    else if (obj.semantics.category.compare("LightObject") == 0)
    {
    	sp[0].y =  precision(center.y - (w / 2 + 0.2));
        sp[1].x =  precision(center.x + (h / 2 + 0.2));    
        sp[2].y =  precision(center.y + (w / 2 + 0.2));
        sp[3].x =  precision(center.x - (h / 2 + 0.2));


    }  
    else if (obj.semantics.category.compare("Unknown") == 0)
    {
    	sp[0].y =  precision(center.y - (h / 2 + 0.3));
        sp[1].x =  precision(center.x + (w / 2 + 0.3));    
        sp[2].y =  precision(center.y + (h / 2 + 0.3));
        sp[3].x =  precision(center.x - (w / 2 + 0.3));

    } 
    ROS_INFO_STREAM(obj.semantics.semantic_positions.position.size());
    for (int i = 0; i < 8; i++)
    {
      	obj.semantics.semantic_positions.position.push_back(sp[i]);
    } 


    
		
}

void DummyPerception::computeValues()
{
	semantic_map::Object object, empty;
	
	Vector2 center;

	object_list_.clear(); 

	for (int i = 0; i < objects_state_temp_.model_name.size(); i++)
	{	
		object = empty;
		std::string instance = objects_state_temp_.model_name[i];

	/*	if (instance.compare(0, 4,"wall") == 0)
		{
			
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);
    		

			object.semantics.category = "StructuralObject";
			object.semantics.sub_category = "Wall";

			if (objects_state_temp_.size[i].x > objects_state_temp_.size[i].y)
			{
				computeBbAndSp(center, 0.3, 3.85, 0, object);
			}
			else if (objects_state_temp_.size[i].x < objects_state_temp_.size[i].y)
			{
				computeBbAndSp(center, 3.85, 0.3, 0, object);
			}
			//computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 	
		}*/

		if (instance.compare(0, 5,"couch") == 0)
		{
			
			object.instance.name = objects_state_temp_.model_name[i];
			
			object.geometry.pose.position.x = precision(objects_state_temp_.pose[i].position.x);			
			object.geometry.pose.position.y = precision(objects_state_temp_.pose[i].position.y);
			object.geometry.pose.position.z = precision(objects_state_temp_.pose[i].position.z);
			object.geometry.pose.orientation.x = precision(objects_state_temp_.pose[i].orientation.x);			
			object.geometry.pose.orientation.y = precision(objects_state_temp_.pose[i].orientation.y);
			object.geometry.pose.orientation.z = precision(objects_state_temp_.pose[i].orientation.z);
			object.geometry.pose.orientation.w = precision(objects_state_temp_.pose[i].orientation.w);
			
    		center.x =  precision(objects_state_temp_.pose[i].position.x);
    		center.y =  precision(objects_state_temp_.pose[i].position.y);
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "Couch";

			
			computeBbAndSp(center, objects_state_temp_.size[i].y, objects_state_temp_.size[i].x, angle, object);

			object_list_.push_back(object); 


		}


		if (instance.compare(0, 8,"table_tv") == 0)
		{
			
			object.instance.name = objects_state_temp_.model_name[i];
			
			object.geometry.pose.position.x = precision(objects_state_temp_.pose[i].position.x);			
			object.geometry.pose.position.y = precision(objects_state_temp_.pose[i].position.y);
			object.geometry.pose.position.z = precision(objects_state_temp_.pose[i].position.z);
			object.geometry.pose.orientation.x = precision(objects_state_temp_.pose[i].orientation.x);			
			object.geometry.pose.orientation.y = precision(objects_state_temp_.pose[i].orientation.y);
			object.geometry.pose.orientation.z = precision(objects_state_temp_.pose[i].orientation.z);
			object.geometry.pose.orientation.w = precision(objects_state_temp_.pose[i].orientation.w);

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "TvTable";

			computeBbAndSp(center, objects_state_temp_.size[i].y, objects_state_temp_.size[i].x, angle, object);

			object_list_.push_back(object); 
		}

		/*else if (instance.compare(0, 2,"tv") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "TV";

			computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 		
		}

		else if (instance.compare(0, 7,"cabinet") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "Cabinet";

			computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 
		}

		if (instance.compare(0, 12,"chair_wicker") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "LightObject";
			object.semantics.sub_category = "ChairWicker";

			computeBbAndSp(center, objects_state_temp_.size[i].y, objects_state_temp_.size[i].x, angle, object);

			object_list_.push_back(object); 	
		}

		else if (instance.compare(0, 9,"arm_chair") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "CouchChair";

			computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 	
		}

		if (instance.compare(0, 5,"stool") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "LightObject";
			object.semantics.sub_category = "Stool";

			computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 	
		}*/

		if (instance.compare(0, 14,"corrogated_box") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];

			object.geometry.pose.position.x = precision(objects_state_temp_.pose[i].position.x);			
			object.geometry.pose.position.y = precision(objects_state_temp_.pose[i].position.y);
			object.geometry.pose.position.z = precision(objects_state_temp_.pose[i].position.z);
			object.geometry.pose.orientation.x = precision(objects_state_temp_.pose[i].orientation.x);			
			object.geometry.pose.orientation.y = precision(objects_state_temp_.pose[i].orientation.y);
			object.geometry.pose.orientation.z = precision(objects_state_temp_.pose[i].orientation.z);
			object.geometry.pose.orientation.w = precision(objects_state_temp_.pose[i].orientation.w);




			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "LightObject";
			object.semantics.sub_category = "Box";

			computeBbAndSp(center, objects_state_temp_.size[i].y, objects_state_temp_.size[i].x, angle, object);

			object_list_.push_back(object); 	
		}

	/*	if (instance.compare(0, 4,"wall") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "StructuralObject";
			object.semantics.sub_category = "Wall";

			if (objects_state_temp_.pose[i].position.x > objects_state_temp_.pose[i].position.y)
			{
				computeBbAndSp(center, objects_state_temp_.size[i].x + 0.7, objects_state_temp_.size[i].y + 0.35, 0, object);
			}
			else
			{
				computeBbAndSp(center, objects_state_temp_.size[i].x + 0.35, objects_state_temp_.size[i].y + 0.7, 0, object);

			}

			object_list_.push_back(object); 	
		}*/
        
		/*if (instance.compare(0, 10,"box_wicker") == 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			
    		center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "LightObject";
			object.semantics.sub_category = "Box";

			computeBbAndSp(center, objects_state_temp_.size[i].y, objects_state_temp_.size[i].x, angle, object);

			object_list_.push_back(object); 	
		}

		else if(instance.compare(0, 12,"ground_plane") != 0 && instance.compare(0, 5,"world") != 0 
			&& instance.compare(0, 4,"p3dx") != 0)
		{
			object.instance.name = objects_state_temp_.model_name[i];
			object.geometry.pose = objects_state_temp_.pose[i];

			center.x = objects_state_temp_.pose[i].position.x;
    		center.y = objects_state_temp_.pose[i].position.y;
    		double angle = (acos (objects_state_temp_.pose[i].orientation.w) * 2) * (180.0 / PI);

			object.semantics.category = "Unknown";
			object.semantics.sub_category = "Unknown";

			computeBbAndSp(center, objects_state_temp_.size[i].x, objects_state_temp_.size[i].y, angle, object);

			object_list_.push_back(object); 	
		}*/

		
	}

}

void DummyPerception::objectsStateCallback(const perception::ObjectsState::ConstPtr& msg)
{
	boost::unique_lock<mutex_t> lock(*access_);

	objects_state_temp_ = *msg;

}

double DummyPerception::precision(double number)
{
	int temp = (number*100);
	double precise_number = temp / 100.0;

	return precise_number;
}




