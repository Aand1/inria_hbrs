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

#ifndef SEMANTIC_MAP_H_
#define SEMANTIC_MAP_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp> 
#include "yamlcpp_convert.h" 
#include <fstream> 
#include <semantic_map/ObjectList.h>
#include <semantic_map/Object.h>
#include <semantic_map/Region.h> 
#include <semantic_map/GeometricProperties.h>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <list>


namespace semantic_map{
	/*
	* @brief : Define struct data types to store information about objects 
	*/

	class SemanticMap
	{
	public:
	    SemanticMap(const ros::NodeHandle &nh);

	    ~SemanticMap();

	    void createMap();

	 	void updateMap(std::list<Object>& object_list);

	 	void deleteMap(); 

	 	void clearMap();

	 	std::list<Object>& getSemanticMap();

	 	void getObject(semantic_map::Object& object); 

	 	void getObjects(std::list<Object>& objects, std::string object_type); 

	 	void getRegions(std::list<Region>& regions); 

	 	std::list<Object>& getSemanticMapDynamic();
	 	void getObjectDynamic(semantic_map::Object& object); 
	 	void getObjectsDynamic(std::list<Object>& objects, std::string object_type); 

	 	std::list<Object>& getSemanticMapStatic();
	 	void getObjectsStatic(std::list<Object>& objects, std::string object_type); 

	 	void getAllObjects(std::list<Object>& objects);


	 	// Provide a typedef to ease future code maintenance
	    typedef boost::recursive_mutex mutex_t;
	    
	/*    mutex_t* getMutex()
	    {
	        return access_;
	    }*/

	private:

		ros::NodeHandle nh_;

		mutex_t* access_;

		std::list<Object> object_list_;

		semantic_map::Object object;

		//YAML::Node* node_so, *node_do;

		//std::ofstream mapfile_so, mapfile_do;
		//std::string filename_so, filename_do;


		YAML::Node* node_so, *node_ho, *node_lo;

		std::ofstream mapfile_so, mapfile_ho, mapfile_lo;
		std::string filename_so, filename_ho, filename_lo;


	};
}
#endif