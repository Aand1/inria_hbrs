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
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <string.h>
#include <math.h>

//#include "math/Vector3.hh"
using namespace gazebo;
using namespace physics;

#define PI 3.14159265

DummyPerception::DummyPerception(const ros::NodeHandle &nh) : nh_(nh), map_update_thread_(NULL), map_create(true)
{
    access_ = new mutex_t();  
	//gms_c = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	//ros::Subscriber sub = nh_.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1000,  boost::bind(DummyPerception::modelStateCallback, this, _1));
    semantic_map = new semantic_map::SemanticMap(nh_);
    //semantic_map::SemanticMap semantic_map(nh_) ;

	gazebo_model_state_sub = nh_.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1000, boost::bind(&DummyPerception::modelStateCallback, this, _1));

	//semantic_map->createMap();
	//semantic_map->updateMap();

	double map_update_frequency = 0.2;
    map_update_thread_shutdown_ = false;
    object_range_ = 3.0;

    map_update_thread_ = new boost::thread(boost::bind(&DummyPerception::updateMapLoop, this, map_update_frequency));
}

void DummyPerception::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	boost::unique_lock<mutex_t> lock(*access_);

	ms_temp = *msg;
	//assignValues(ms_temp);
    	    	
    //ROS_INFO_STREAM(object_list_.size());

	//lock.unlock();

	//ROS_INFO_STREAM(Couch.width);
	//list<gazebo_msgs::ModelStates>::iterator msg_it;
	
	
	//gazebo::physics::WorldPtr world = gazebo::physics::get_world("p3dx");
	//gazebo::physics::ModelPtr model =  "p3dx";

/*	for (int i = 0; i < msg->name.size(); i++)
	{
		if (msg->name[i].compare("ground_plane") != 0)// != 0 && msg->name[i].compare("world") && msg->name[i].compare("p3dx"))
    	{
    		ms.name.push_back(msg->name[0]);
    		//ms_temp = *msg;
    		// = ms_temp.name[0];
    		//ms.name[0] = msg->name[0];
    		//std::string instance = msg->name[0];
    	//	ms.name[0] = *msg->name[0];
    		//ROS_INFO_STREAM(instance);
 
    	}


	}

	if (map_create)
	{
		ROS_INFO_STREAM("Creating map");
		//semantic_map->createMap(ms);
		map_create = false;
	}
    //*ms = msg;
    //semantic_map->updateMap(msg);
    //gazebo_msgs::ModelState model_state;
    //model_state.model_name = ms.name[0];
    //semantic_map->updateMap();
    //ROS_INFO_STREAM(ms.name[0]);
    //for (int i=0;i<ms.name.size();i++) 
    //{
  		//std::cout << primes[i].as<int>() << "\n";
	//}*/
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
	//semantic_map.updateMap();

//    semantic_map->updateMap();
	//semantic_map->updateMap(ms);

}

void DummyPerception::updateMapLoop(double frequency)
{
	if (frequency == 0.0)
    	return;

    ros::NodeHandle nh;
    ros::Rate r(frequency);
  
    while (nh.ok() && !map_update_thread_shutdown_)
    {
    	boost::unique_lock<mutex_t> lock(*access_);
    	assignValues(ms_temp);
    	//ROS_INFO_STREAM(object_list_.size());
    	lock.unlock();
    	    	
    	//ROS_INFO_STREAM(object_list_.size());
    	semantic_map->updateMap(object_list_);
    	r.sleep();
    
    	// make sure to sleep for the remainder of our cycle time
    	if (r.cycleTime() > ros::Duration(1 / frequency))
    	{
      		ROS_WARN("Semantic Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
      	}
  
  	}
}

void DummyPerception::computeBB(const Vector2& center, const double w, const double h, double angle, semantic_map::Object& obj)
{
		Vector2 X( cos(angle), sin(angle));
		Vector2 Y(-sin(angle), cos(angle));

		X *= w / 2;
        Y *= h / 2;

        corner[0] = center - X - Y;
        corner[1] = center + X - Y;
        corner[2] = center + X + Y;
        corner[3] = center - X + Y;
 
        for (int i = 0; i < 4; i++)
        {
        	obj.geometry.bounding_box.vertices[i].x = corner[i].x; 
        	obj.geometry.bounding_box.vertices[i].y = corner[i].y;  
    	}
}

void DummyPerception::assignValues(gazebo_msgs::ModelStates& modelstates)
{
	semantic_map::Object object, empty;
	Vector2 center;

	object_list_.clear(); 

	for (int i = 0; i < modelstates.name.size(); i++)
	{	
		object = empty;
		std::string instance = modelstates.name[i];

		if (instance.compare(0, 5,"couch") == 0)
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			
    		center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 2.0, 1.0, angle, object);
			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "Couch";

			object_list_.push_back(object); 
		}

		else if (instance.compare(0, 8,"table_tv") == 0)
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			
    		center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 1.25, 0.5, angle, object);
			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "TvTable";

			object_list_.push_back(object); 
		}

		else if (instance.compare(0, 2,"tv") == 0)
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			
    		center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 1.0, 0.25, angle, object);
			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "TV";

			object_list_.push_back(object); 		
		}

		else if (instance.compare(0, 7,"cabinet") == 0)
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			
    		center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 1.875, 0.45, angle, object);
			object.semantics.category = "HeavyObject";
			object.semantics.sub_category = "Cabinet";

			object_list_.push_back(object); 
		}

		else if (instance.compare(0, 12,"chair_wicker") == 0)
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			
    		center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 0.5, 0.5, angle, object);
			object.semantics.category = "LightObject";
			object.semantics.sub_category = "ChairWicker";

			object_list_.push_back(object); 	
		}

		/*else
		{
			object.instance.name = modelstates.name[i];
			object.geometry.pose = modelstates.pose[i];

			center.x = modelstates.pose[i].position.x;
    		center.y = modelstates.pose[i].position.y;
    		double angle = (acos (modelstates.pose[i].orientation.w) * 2) * (180.0 / PI);
			computeBB(center, 1.0, 1.0, angle, object);
			object.semantics.category = "Unknown";
			object.semantics.sub_category = "Unknown";

			object_list_.push_back(object); 	
		}*/

		
	}

}

/*
void DummyPerception::queryGazebo()
{

	if(gms_c.exists()) 
    {
    	gms.request.model_name="box1";
    	
    	if (gms_c.call(gms))
    	{
    		ROS_INFO_STREAM("Response recieved");

    		Vector2 center;
    		center.x = gms.response.pose.position.x;
    		center.y = gms.response.pose.position.y;
    		double theta = (acos (gms.response.pose.orientation.w) * 2) * (180.0 / PI);

    		double width = 1.0;
    		double height = 1.0;    	
    	
    	}

    	else
    	{
    		ROS_INFO_STREAM("Response FAILED");
    	}


	}
	
}


double DummyPerception::distanceCalculate(double x1, double y1, double x2, double y2)
{
    double x = x1 - x2;
    double y = y1 - y2;
    double dist;

    dist = pow(x,2)+pow(y,2);           //calculating distance by euclidean formula
    dist = sqrt(dist);                  //sqrt is function in math.h

    return dist;
}
*/


