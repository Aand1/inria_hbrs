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
 *		            Dr. Anne Spalanzani (Inria)
 *********************************************************************/
#include "yaml-cpp/yaml.h"
#include <semantic_map/semantic_map.h>
#include <semantic_map/ObjectInstance.h>
#include <geometry_msgs/Vector3.h>
#include <semantic_map/BoundingBox.h>
#include <semantic_map/SemanticPositions.h>
#include <semantic_map/GeometricProperties.h>
#include <semantic_map/SemanticProperties.h>
#include <semantic_map/Object.h>
#include <semantic_map/Region.h>
#include <iostream>
#include <string>
#include <vector> 

/*
* @brief : Define struct data types to store information about objects 
*/

namespace YAML 
{
  template<>
	struct convert<semantic_map::ObjectInstance> 
	{
  		static Node encode(const semantic_map::ObjectInstance& rhs) 
  		{
    		Node node;

    		node = rhs.name;

    		return node;
  		}

  		static bool decode(const Node& node, semantic_map::ObjectInstance& rhs) 
	    {

    		rhs.name = node.as<std::string>();
    		
    		return true;
  		}
	};

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

  		static bool decode(const Node& node, geometry_msgs::Vector3& rhs) 
      {
        //if(node.size() != 3) 
        //{
        //  return false;
        //}

        rhs.x = node[0].as<double>();
        rhs.y = node[1].as<double>();
        rhs.z = node[2].as<double>();
        return true;
      }
	};

	template<>
	struct convert<semantic_map::BoundingBox> 
	{
  		static Node encode(const semantic_map::BoundingBox& rhs) 
  		{
    		Node node;

    		node["vertex-1"] = rhs.vertices[0];
    		node["vertex-2"] = rhs.vertices[1];
    		node["vertex-3"] = rhs.vertices[2];
    		node["vertex-4"] = rhs.vertices[3];

		    return node;
  		}

  		static bool decode(const Node& node, semantic_map::BoundingBox& rhs) 
	    {

    		rhs.vertices[0] = node["vertex-1"].as<geometry_msgs::Vector3>();
    		rhs.vertices[1] = node["vertex-2"].as<geometry_msgs::Vector3>();
    		rhs.vertices[2] = node["vertex-3"].as<geometry_msgs::Vector3>();
    		rhs.vertices[3] = node["vertex-4"].as<geometry_msgs::Vector3>();
    		
    		return true;
  		}
	};

  template<>
  struct convert<semantic_map::SemanticPositions> 
  {
      static Node encode(const semantic_map::SemanticPositions& rhs) 
      {
        Node node;

        node["position-1"] = rhs.position[0];
        node["position-2"] = rhs.position[1];
        node["position-3"] = rhs.position[2];
        node["position-4"] = rhs.position[3];

        node["position-5"] = rhs.position[4];
        node["position-6"] = rhs.position[5];
        node["position-7"] = rhs.position[6];
        node["position-8"] = rhs.position[7];

        return node;
      }

      static bool decode(const Node& node, semantic_map::SemanticPositions& rhs) 
      {
        
        rhs.position.resize(8);   
        rhs.position[0] = node["position-1"].as<geometry_msgs::Vector3>();
        rhs.position[1] = node["position-2"].as<geometry_msgs::Vector3>();
        rhs.position[2] = node["position-3"].as<geometry_msgs::Vector3>();
        rhs.position[3] = node["position-4"].as<geometry_msgs::Vector3>();

        rhs.position[4] = node["position-5"].as<geometry_msgs::Vector3>();
        rhs.position[5] = node["position-6"].as<geometry_msgs::Vector3>();
        rhs.position[6] = node["position-7"].as<geometry_msgs::Vector3>();
        rhs.position[7] = node["position-8"].as<geometry_msgs::Vector3>();


        
        return true;
      }
  };

	template<>
	struct convert<semantic_map::GeometricProperties> 
	{
		static Node encode(const semantic_map::GeometricProperties& rhs) 
	    {
	        Node node;
	    
	   	  	node["pose"]["position"]["x"] = rhs.pose.position.x;
	      	node["pose"]["position"]["y"] = rhs.pose.position.y;
	      	node["pose"]["position"]["z"] = rhs.pose.position.z;
	      	node["pose"]["orientation"]["x"] = rhs.pose.orientation.x;
	      	node["pose"]["orientation"]["y"] = rhs.pose.orientation.y;
	      	node["pose"]["orientation"]["z"] = rhs.pose.orientation.z;
	      	node["pose"]["orientation"]["w"] = rhs.pose.orientation.w;
	      	
	      	node["bounding_box"] = rhs.bounding_box;

	      	return node;
	    }

	    static bool decode(const Node& node, semantic_map::GeometricProperties& rhs) 
	    {
    		
    		rhs.pose.position.x = node["pose"]["position"]["x"].as<double>();
    		rhs.pose.position.y = node["pose"]["position"]["y"].as<double>();
    		rhs.pose.position.z = node["pose"]["position"]["z"].as<double>();
    		rhs.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
    		rhs.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
    		rhs.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
    		rhs.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();

    		rhs.bounding_box = node["bounding_box"].as<semantic_map::BoundingBox>();
    		
    		return true;
  		}

	};

	template<>
	struct convert<semantic_map::SemanticProperties> 
	{
  		static Node encode(const semantic_map::SemanticProperties& rhs) 
  		{
    		Node node;

    		node["category"] = rhs.category;
    		node["sub_category"] = rhs.sub_category;
        node["semantic_positions"] = rhs.semantic_positions;
        node["affordance"] = rhs.affordance;

    		return node;
  		}

  		static bool decode(const Node& node, semantic_map::SemanticProperties& rhs) 
	    {
    		
    		rhs.category = node["category"].as<std::string>();
    		rhs.sub_category = node["sub_category"].as<std::string>();
        rhs.semantic_positions = node["semantic_positions"].as<semantic_map::SemanticPositions>();
        rhs.affordance = node["affordance"].as<std::string>();
    		
    		return true;
  		}
	};

	template<>
	struct convert<semantic_map::Object> 
	{
  		static Node encode(const semantic_map::Object& rhs) 
  		{
    		Node node;

    		node["instance"] = rhs.instance; 
    		node["geometry"] = rhs.geometry;
    		node["semantics"] = rhs.semantics;

    		return node;
  		} 

  		static bool decode(const Node& node, semantic_map::Object& rhs) 
	    {
    		
    		rhs.instance = node["instance"].as<semantic_map::ObjectInstance>();
    		rhs.geometry = node["geometry"].as<semantic_map::GeometricProperties>();
    		rhs.semantics = node["semantics"].as<semantic_map::SemanticProperties>();
    		
    		return true;
  		} 
	};


  template<>
  struct convert<semantic_map::Region> 
  {
      static Node encode(const semantic_map::Region& rhs) 
      {
        Node node;

        node["instance"] = rhs.instance.name; 
        node["geometry"] = rhs.geometry;

        node["semantics"]["category"] = rhs.semantics.category;
        node["semantics"]["sub_category"] = rhs.semantics.sub_category;
        node["semantics"]["semantic_positions"]["position-1"] = rhs.semantics.semantic_positions.position[0];        

        return node;
      } 

      static bool decode(const Node& node, semantic_map::Region& rhs) 
      {
        
        rhs.instance.name = node["instance"].as<std::string>();
       
        rhs.geometry = node["geometry"].as<semantic_map::GeometricProperties>();

        rhs.semantics.category = node["semantics"]["category"].as<std::string>();
        rhs.semantics.sub_category = node["semantics"]["sub_category"].as<std::string>();
        rhs.semantics.semantic_positions.position.resize(1);   
        rhs.semantics.semantic_positions.position[0] = node["semantics"]["semantic_positions"]["position-1"].as<geometry_msgs::Vector3>();
        
        return true;
      } 
  };
}
/*struct Position 
{ 
	double x; 
	double y; 
	double z; 
};

struct Orientation 
{ 
	double x; 
	double y; 
	double z; 
	double w; 
};

struct Pose
{
    Position position;
    Orientation orientation;
};

struct Vertex
{ 
	double x; 
	double y; 
	double z; 
};

struct BoundingBox
{
    Vertex one;
    Vertex two;
    Vertex three;
    Vertex four;
};

struct Geometry
{
    Pose pose;
    BoundingBox bb;
};

struct Semantics
{
    std::string category;
    std::string sub_category;
};

struct Object
{
    std::string instance;
    Geometry geometry;
    Semantics semantics;

};

*/
/*
* @brief : Specialize the YAML::convert<> template class for your data types
*/
/*
namespace YAML 
{
    template<>
    struct convert<Vertex> 
    {
        static Node encode(const Vertex& rhs) 
        {
    		Node node;
    		node.push_back(rhs.x);
    		node.push_back(rhs.y);
    		node.push_back(rhs.z);
    		return node;
  		}

  		static bool decode(const Node& node, Vertex& rhs) 
  		{
    		if(!node.IsSequence() || node.size() != 3) 
    		{
      			return false;
    		}

    		rhs.x = node[0].as<double>();
    		rhs.y = node[1].as<double>();
    		rhs.z = node[2].as<double>();
    		return true;
  		}		
	};


	template<>
	struct convert<BoundingBox> 
	{
  		static Node encode(const BoundingBox& rhs) 
  		{
    		Node node;

    		node["vertex-1"]["x"] = rhs.one.x;
		    node["vertex-1"]["y"] = rhs.one.y;
		    node["vertex-1"]["z"] = rhs.one.z;

		    node["vertex-2"]["x"] = rhs.two.x;
		    node["vertex-2"]["y"] = rhs.two.y;
		    node["vertex-2"]["z"] = rhs.two.z;

		    node["vertex-3"]["x"] = rhs.three.x;
		    node["vertex-3"]["y"] = rhs.three.y;
		    node["vertex-3"]["z"] = rhs.three.z;

		    node["vertex-4"]["x"] = rhs.four.x;
		    node["vertex-4"]["y"] = rhs.four.y;
		    node["vertex-4"]["z"] = rhs.four.z;

		    return node;
  		}
	};

	template<>
	struct convert<Geometry> 
	{
		static Node encode(const Geometry& rhs) 
	    {
	        Node node;
	    
	   	  	node["pose"]["position"]["x"] = rhs.pose.position.x;
	      	node["pose"]["position"]["y"] = rhs.pose.position.y;
	      	node["pose"]["position"]["z"] = rhs.pose.position.z;
	      	node["pose"]["orientation"]["x"] = rhs.pose.orientation.x;
	      	node["pose"]["orientation"]["y"] = rhs.pose.orientation.y;
	      	node["pose"]["orientation"]["z"] = rhs.pose.orientation.z;
	      	node["pose"]["orientation"]["w"] = rhs.pose.orientation.w;
	      	node["bounding_box"] = rhs.bb;

	      	return node;
	    }
	};

	template<>
	struct convert<Semantics> 
	{
  		static Node encode(const Semantics& rhs) 
  		{
    		Node node;

    		node["category"] = rhs.category;
    		node["sub_category"] = rhs.sub_category;

    		return node;
  		}
	};

	template<>
	struct convert<Object> 
	{
  		static Node encode(const Object& rhs) 
  		{
    		Node node;

    		node["instance"] = rhs.instance; 
			node["geometry"] = rhs.geometry;
			node["semantics"] = rhs.semantics;

    		return node;
  		}
	};

}*/
