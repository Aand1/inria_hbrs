/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Hochschule Bonn-Rhein-Sieg, Germany
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
 *   * Neither the name of Hochschule Bonn-Rhein-Sieg, Germany and ENSTA 
 *     ParisTech, France nor the names of its contributors may be used to 
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

#include <semantic_map/semantic_map.h>
#include "yaml-cpp/yaml.h"
#include <string.h>
#include <iostream>

using namespace std;
using namespace semantic_map;
/*namespace YAML 
{
    template<>
	struct convert<geometry_msgs::Vector3> 
	{
  		static Node encode(const geometry_msgs::Vector3& rhs) 
  		{
    		Node node;

    		node.push_back(rhs.x);
    		node.push_back(rhs.y);
    		node.push_back(rhs.z);

			return node;
  		}
	};
}*/
SemanticMap::SemanticMap(const ros::NodeHandle &nh) 
  : nh_(nh)
{

    //filename_so = "/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/static_objects.yaml";
    //filename_do = "/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/dynamic_objects.yaml";

    filename_so = "/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/structural_objects.yaml";
    filename_ho = "/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/heavy_objects.yaml";
    filename_lo = "/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/light_objects.yaml";

    
    access_ = new mutex_t();   
}

SemanticMap::~SemanticMap()
{
	delete access_;
}

void SemanticMap::createMap()
{
	//boost::unique_lock<mutex_t> lock(*access_);
    gazebo_msgs::ModelStates modelstates;
	  clearMap();

	  std::string instance_name;
    semantic_map::Object object;
    gazebo_msgs::ModelState model_state;
    //std::ofstream fout_so("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/semantic_map/data/static_objects.yaml");
    //std::ofstream fout_do("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/semantic_map/data/dynamic_objects.yaml");


    //ROS_INFO_STREAM(modelstates.name.size());

	for (int i = 0; i < 5/*modelstates.name.size()*/; i++) 
    {
    	//if (modelstates->name[i].compare("ground_plane") != 0 && modelstates->name[i].compare("world") && modelstates->name[i].compare("ground_plane"))
    	{

    	
    			//object.instance.name = modelstates.name[i];
  		//object.geometry.pose = modelstates->pose[0];
  				node_so[i] = object;
  		//node[i] = object.geometry;
  		//node[i] = object.semantics;
  		//node[1] = object;
  		//node[2] = object;
  		//fout << node;
  		//ROS_INFO_STREAM("FIlling map");
  		}
    }
    //fout << node_so;
    //boost::unique_lock<boost::recursive_mutex> scoped_lock(access_);
/*    ROS_INFO_STREAM("Mutex used for createMap");

	YAML::Node node;  // starts out as null 
	//semantic_map::BoundingBox bb;
    geometry_msgs::Vector3 v;
    //Object o1, o2;
    //Geometry g1;
    //node[0]["geometry"] = g1; 
    //Position p;
    //Orientation o;
    //semantic_map::GeometricProperties g1;
    //g1.bounding_box.vertices.reserve(4); 
    semantic_map::Object o1, o2;

    //o1.geometry.bounding_box.vertices[0] = v;
   // o1.geometry.bounding_box.vertices[1] = v;
    //o1.geometry.bounding_box.vertices[2] = v;
   // o1.geometry.bounding_box.vertices[3] = v;

    node[0] = o1; 
    node[1] = o1; 

    //std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/semantic_map/data/semantic_map.yaml");
    fout << node;*/

    //lock.unlock();

}

void SemanticMap::updateMap(std::list<Object>& object_list)
{   
	  boost::unique_lock<mutex_t> lock(*access_);

    if (object_list.size() == 0)
      	return;

	  //clearMap();
    
    list<Object>::iterator obs_it;

  /*  node_so = new YAML::Node[object_list.size()];
    node_do = new YAML::Node[object_list.size()];

  	for (obs_it = object_list.begin(); obs_it != object_list.end(); ++obs_it)
  	{
    	semantic_map::Object& object = *obs_it;

      if (  object.semantics.category.compare("StructuralObject") == 0 )
      {    	
    	    node_so->push_back(object);
      }

      else 
      {
          node_do->push_back(object);
      }
  	}
    
    if ( node_so->size() != 0 )
    {
        mapfile_so.open (filename_so.c_str(), std::fstream::out | std::fstream::trunc);
        mapfile_so << *node_so;
        mapfile_so.close();
    }

    if ( node_do->size() != 0 )
    {
        mapfile_do.open (filename_do.c_str(), std::fstream::out | std::fstream::trunc);
        mapfile_do << *node_do;
        mapfile_do.close();
    }*/

    node_so = new YAML::Node[object_list.size()];
    node_ho = new YAML::Node[object_list.size()];
    node_lo = new YAML::Node[object_list.size()];

    for (obs_it = object_list.begin(); obs_it != object_list.end(); ++obs_it)
    {
      semantic_map::Object& object = *obs_it;

      if (  object.semantics.category.compare("StructuralObject") == 0 )
      {     
          node_so->push_back(object);
      }

      else if ( object.semantics.category.compare("HeavyObject") == 0 )
      {
          node_ho->push_back(object);
      }

      else if ( object.semantics.category.compare("LightObject") == 0 )
      {
          node_lo->push_back(object);
      }
    }

    if ( node_so->size() != 0 )
    {
        mapfile_so.open (filename_so.c_str(), std::fstream::out | std::fstream::trunc);
        mapfile_so << *node_so;
        mapfile_so.close();
    }

    if ( node_ho->size() != 0 )
    {
        mapfile_ho.open (filename_ho.c_str(), std::fstream::out | std::fstream::trunc);
        mapfile_ho << *node_ho;
        mapfile_ho.close();
    }

    if ( node_lo->size() != 0 )
    {
        mapfile_lo.open (filename_lo.c_str(), std::fstream::out | std::fstream::trunc);
        mapfile_lo << *node_lo;
        mapfile_lo.close();
    }
}

void SemanticMap::deleteMap()
{
	boost::unique_lock<mutex_t> lock(*access_);

	//lock.unlock();

}

void SemanticMap::clearMap()
{
	boost::unique_lock<mutex_t> lock(*access_);

	delete[] node_so;

	object_list_.clear(); 

}

std::list<Object>& SemanticMap::getSemanticMap() 
{

	object_list_.clear(); 

  boost::unique_lock<mutex_t> lock(*access_);

	YAML::Node node1 = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/semantic_map.yaml");
	//ROS_INFO_STREAM(node1.size());

	for (int count = 0; count < node1.size(); count++)
	{
		object_list_.push_back(node1[count].as<Object>()); 	
	}
	//ROS_INFO_STREAM(object_list_.size());    

    return object_list_;
}

void SemanticMap::getObject(semantic_map::Object& object) 
{

  boost::unique_lock<mutex_t> lock(*access_);

  semantic_map::Object object_temp;

  YAML::Node node1 = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/semantic_map.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node1.size(); count++)
  {
    //object_list_.push_back(node1[count].as<Object>());  

    object_temp = node1[count].as<Object>();

    if ( !object_temp.instance.name.compare(object.instance.name) )
    {
      object = object_temp;

    }

  }
  //ROS_INFO_STREAM(object_list_.size());    

    
}

void SemanticMap::getObjects(std::list<Object>& objects, std::string object_type) 
{

  boost::unique_lock<mutex_t> lock(*access_);

  objects.clear();

  semantic_map::Object object_temp;

  YAML::Node node1 = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/semantic_map.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node1.size(); count++)
  {
    //object_list_.push_back(node1[count].as<Object>());  

    object_temp = node1[count].as<Object>();


    if ( !object_temp.semantics.category.compare(object_type) )
    {
      //ROS_INFO_STREAM(object_temp.semantics.category);
      objects.push_back(object_temp);  

    }



  }
  //ROS_INFO_STREAM(object_list_.size());    

    
}

std::list<Object>& SemanticMap::getSemanticMapDynamic() 
{

  boost::unique_lock<mutex_t> lock(*access_);
  object_list_.clear(); 

  YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/dynamic_objects.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node.size(); count++)
  {
    object_list_.push_back(node[count].as<Object>());  
  }
  //ROS_INFO_STREAM(object_list_.size());    

  return object_list_;
}

void SemanticMap::getObjectDynamic(semantic_map::Object& object) 
{

  boost::unique_lock<mutex_t> lock(*access_);

  semantic_map::Object object_temp;

  YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/dynamic_objects.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node.size(); count++)
  {
    //object_list_.push_back(node1[count].as<Object>());  

    object_temp = node[count].as<Object>();

    if ( !object_temp.instance.name.compare(object.instance.name) )
    {
      object = object_temp;

    }

  }
  //ROS_INFO_STREAM(object_list_.size());    

    
}
/////////////////////////
void SemanticMap::getObjectsDynamic(std::list<Object>& objects, std::string object_type) 
{

  boost::unique_lock<mutex_t> lock(*access_);

  objects.clear();

  semantic_map::Object object_temp;

  YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/dynamic_objects.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node.size(); count++)
  {
    //object_list_.push_back(node1[count].as<Object>());  

    object_temp = node[count].as<Object>();


    if ( !object_temp.semantics.category.compare(object_type) )
    {
      //ROS_INFO_STREAM(object_temp.semantics.category);
      objects.push_back(object_temp);  

    }


  }   
}

std::list<Object>& SemanticMap::getSemanticMapStatic() 
{

  object_list_.clear(); 

  boost::unique_lock<mutex_t> lock(*access_);

  YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/static_objects.yaml");
  //ROS_INFO_STREAM(node1.size());

  for (int count = 0; count < node.size(); count++)
  {
    object_list_.push_back(node[count].as<Object>());  
  }
  //ROS_INFO_STREAM(object_list_.size());    

    return object_list_;
}

void SemanticMap::getObjectsStatic(std::list<Object>& objects, std::string object_type) 
{

    boost::unique_lock<mutex_t> lock(*access_);

    objects.clear();

  /*  semantic_map::Object object_temp;

    YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/static_objects.yaml");
    //ROS_INFO_STREAM(node1.size());

    for (int count = 0; count < node.size(); count++)
    {

        object_temp = node[count].as<Object>();


        if ( !object_temp.semantics.category.compare(object_type) )
        {
            //ROS_INFO_STREAM(object_temp.semantics.category);
            objects.push_back(object_temp);  

        }
    } */
    semantic_map::Object object_temp;
    if (  object_type.compare("StructuralObject") == 0 )
    {    
        YAML::Node node_so = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/structural_objects.yaml"); 

        for (int count = 0; count < node_so.size(); count++)
        {
            object_temp = node_so[count].as<Object>();
            objects.push_back(object_temp);
        }
        
    } 

    if (  object_type.compare("HeavyObject") == 0 )
    { 
        YAML::Node node_ho = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/heavy_objects.yaml");    
        for (int count = 0; count < node_ho.size(); count++)
        {
            object_temp = node_ho[count].as<Object>();
            objects.push_back(object_temp);
        }
    } 

    else if (  object_type.compare("LightObject") == 0 )
    {  
        YAML::Node node_lo = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/light_objects.yaml"); 
        for (int count = 0; count < node_lo.size(); count++)
        {
            object_temp = node_lo[count].as<Object>();
            objects.push_back(object_temp);
        }
        
    }    

}

void SemanticMap::getRegions(std::list<Region>& regions)
{
    boost::unique_lock<mutex_t> lock(*access_);

    regions.clear();

    YAML::Node node1 = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/regions.yaml");

    for (int count = 0; count < node1.size(); count++)
    {
        regions.push_back( node1[count].as<Region>() );  

    } 

}

void SemanticMap::getAllObjects(std::list<Object>& objects)
{

    boost::unique_lock<mutex_t> lock(*access_);

    objects.clear();

    YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/semantic_navigation/knowledge_base/semantic_map/data/dynamic_objects.yaml");

    semantic_map::Object object_temp;

    for (int count = 0; count < node.size(); count++)
    {
        object_temp = node[count].as<Object>();
        objects.push_back(object_temp); 
    }
}

/////////////////////////////////////////////////////////////////////////////////




