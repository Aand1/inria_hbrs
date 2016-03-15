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

#include <semantic_costmap/dynamic_objects.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE; 

namespace semantic_navigation_layers
{
	DynamicObjects::DynamicObjects(costmap_2d::Costmap2D* parent, ros::NodeHandle& nh, tf::TransformListener *tf, std::string global_frame, bool rolling_window, std::string object_type) 
		: nh_(nh), tf_(tf), layer_grid_(NULL), object_type_(object_type),
		  combination_method_(1),
		  max_obstacle_height_(5.0)  	
	{
		initialize(parent, global_frame, rolling_window);
	}

	DynamicObjects::~DynamicObjects()
	{
		
	}

	void DynamicObjects::initialize(costmap_2d::Costmap2D* parent, std::string global_frame, bool rolling_window)
	{
		layer_grid_ = parent;
    	rolling_window_ = rolling_window;

    	global_frame_ = global_frame;

	    bool track_unknown_space = false;
	  
	    if(track_unknown_space)
	      default_value_ = NO_INFORMATION;
	    else
	      default_value_ = FREE_SPACE;

	    semantic_map_query = new semantic_map::SemanticMap(nh_);

	    current_ = true;
	    enabled_ = true;

    		
        // Initialize parameters
    	double transform_tolerance;
  		nh_.param("transform_tolerance", transform_tolerance, 0.2);

  		std::string topics_string;
	  	// get the topics that we'll subscribe to from the parameter server
	  	nh_.param("observation_sources", topics_string, std::string(""));
	  	ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());

	  	// get our tf prefix
	  	ros::NodeHandle prefix_nh;
	  	const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

	  	// now we need to split the topics based on whitespace which we can use a stringstream for
	  	std::stringstream ss(topics_string);

	  	std::string source;

	  	while (ss >> source)
  		{
    		ros::NodeHandle source_node(nh_, source);

    		// get the parameters for the specific topic
		    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
		    std::string topic, sensor_frame, data_type;
		    bool inf_is_valid, clearing, marking;

		    source_node.param("topic", topic, source);
		    source_node.param("sensor_frame", sensor_frame, std::string("/lms100"));
		    source_node.param("observation_persistence", observation_keep_time, 0.0);
		    source_node.param("expected_update_rate", expected_update_rate, 0.0);
		    source_node.param("data_type", data_type, std::string("LaserScan"));
		    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
		    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
		    source_node.param("inf_is_valid", inf_is_valid, false);
		    source_node.param("clearing", clearing, true);
		    source_node.param("marking", marking, true);

		/*    observation_keep_time = 0.0;
		    expected_update_rate = 0.0;
		    min_obstacle_height = 0.00;
		    max_obstacle_height = 5.0;

		    topic = "/scan";
		    sensor_frame = "/lms100";
		    data_type = "LaserScan";

		    clearing = true;
    		marking = true;*/

		    if (!sensor_frame.empty())
		    {
		      sensor_frame = tf::resolve(tf_prefix, sensor_frame);
		    }

		    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
		    {
		      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
		      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
		    }

		    std::string raytrace_range_param_name, obstacle_range_param_name;

		    // get the obstacle range for the sensor
		    double obstacle_range = 5.0;
		/*    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
		    {
		      source_node.getParam(obstacle_range_param_name, obstacle_range);
		    }*/

		    // get the raytrace range for the sensor
		    double raytrace_range = 5.0;
		/*    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
		    {
		      source_node.getParam(raytrace_range_param_name, raytrace_range);
		    }*/

		    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
		              sensor_frame.c_str());


		    // create an observation buffer
		    observation_buffers_.push_back( boost::shared_ptr <costmap_2d::ObservationBuffer > 
    		(new costmap_2d::ObservationBuffer(topic, observation_keep_time, expected_update_rate, 
    		min_obstacle_height, max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_, 
    		sensor_frame, transform_tolerance)));


		    // check if we'll add this buffer to our marking observation buffers
		    if (marking)
		      marking_buffers_.push_back(observation_buffers_.back());

		    // check if we'll also add this buffer to our clearing observation buffers
		    
		    if (clearing)
		      clearing_buffers_.push_back(observation_buffers_.back());

		    ROS_DEBUG(
		        "Created an observation buffer for source %s, topic %s, global frame: %s, "
		        "expected update rate: %.2f, observation persistence: %.2f",
		        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

		    boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan> > 
	    	sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan", 50));

	    	boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan> > 
	    	filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

	    	filter->registerCallback(boost::bind(&DynamicObjects::laserScanValidInfCallback, this, _1, observation_buffers_.back()));

		    observation_subscribers_.push_back(sub);
		    observation_notifiers_.push_back(filter);
		    observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

		    if (sensor_frame != "")
		    {
		      	std::vector < std::string > target_frames;
		      	target_frames.push_back(global_frame_);
		      	target_frames.push_back(sensor_frame);
		      	observation_notifiers_.back()->setTargetFrames(target_frames);
		    }


		}

	/*	// Assign values to parameters (For now manually)
		double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height, transform_tolerance;
		std::string topic, sensor_frame, data_type;
    	bool clearing, marking;

		topic = "/scan";
		sensor_frame = "/lms100";
    	data_type = "LaserScan";

		observation_keep_time = 0.0;
    	expected_update_rate = 0.0;
    	min_obstacle_height = 0.00;
    	max_obstacle_height = 5.0;
    	transform_tolerance = 0.2;

    	clearing = true;
    	marking = true;

    	double obstacle_range = 3.0;
    	double raytrace_range = 10.0;

    	if (!sensor_frame.empty())
   		{
			sensor_frame = tf::resolve(tf_prefix, sensor_frame);
   		}

		//create an observation buffer
    	observation_buffers_.push_back( boost::shared_ptr <costmap_2d::ObservationBuffer > 
    		(new costmap_2d::ObservationBuffer(topic, observation_keep_time, expected_update_rate, 
    		min_obstacle_height, max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_, 
    		sensor_frame, transform_tolerance)));

    	//check if we'll add this buffer to our marking observation buffers
  		if (marking)
    		marking_buffers_.push_back(observation_buffers_.back());
  		//check if we'll also add this buffer to our clearing observation buffers
  		if (clearing)
    		clearing_buffers_.push_back(observation_buffers_.back()); 


    	boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan> > 
    	sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, "/scan", 50));

    	boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan> > 
    	filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

    	filter->registerCallback(boost::bind(&DynamicObjects::laserScanValidInfCallback, this, _1, observation_buffers_.back()));

	    observation_subscribers_.push_back(sub);
	    observation_notifiers_.push_back(filter);
	    observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

	    if (sensor_frame != "")
	    {
	      	std::vector < std::string > target_frames;
	      	target_frames.push_back(global_frame_);
	      	target_frames.push_back(sensor_frame);
	      	observation_notifiers_.back()->setTargetFrames(target_frames);
	    }*/
	    

	}

	void DynamicObjects::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
	{
		
        //matchSize();       
		std::list<semantic_map::Object> object_list_;
		//ROS_INFO_STREAM(object_type);

		semantic_map_query->getObjectsDynamic(object_list_, object_type_);
		
		useExtraBounds(min_x, min_y, max_x, max_y);
    
		bool current = true;
  		std::vector<costmap_2d::Observation> observations, clearing_observations;

  		// get the marking observations
  		current = current && getMarkingObservations(observations);

  		// get the clearing observations
  		current = current && getClearingObservations(clearing_observations);

  		// update the global current status
  		current_ = current;

  		//raytrace freespace
    	for (unsigned int i = 0; i < clearing_observations.size(); ++i)
    	{
      		//ROS_INFO_STREAM("raytrace running");
      		raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    	}

    	//place the new obstacles into a priority queue... each with a priority of zero to begin with
    	for (std::vector<costmap_2d::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    	{
    		const costmap_2d::Observation& obs = *it;
      		const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);
      		double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

      		for (unsigned int i = 0; i < cloud.points.size(); ++i)
      		{
      			
      			double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;
		        //if the obstacle is too high or too far away from the robot we won't add it
		        if (pz > max_obstacle_height_)
		        {
		          ROS_DEBUG("The point is too high");
		          continue;
		        }
		        //compute the squared distance from the hitpoint to the pointcloud's origin
        		double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) + (pz - obs.origin_.z) * (pz - obs.origin_.z);

        		//if the point is far enough away... we won't consider it
        		if (sq_dist >= sq_obstacle_range)
        		{
          			ROS_DEBUG("The point is too far away");
          			continue;
        		}
        		//now we need to compute the map coordinates for the observation
        		unsigned int mx, my;
        		if (!worldToMap(px, py, mx, my))
        		{
          			ROS_DEBUG("Computing map coords failed");
            		continue;
        		}
        		//check if the point belongs to the specific object category
		        if (!inObjectBoundingBox(mx, my, object_list_))
		        {
		          
		          continue;

		        }
        		unsigned int index = getIndex(mx, my);
        		costmap_[index] = costmap_2d::LETHAL_OBSTACLE;
        		touch(px, py, min_x, min_y, max_x, max_y);   
                
       		}

    	}

	}

	bool DynamicObjects::getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const
	{
		bool current = true;
		// get the marking observations
		for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
		{
		    marking_buffers_[i]->lock();
		    marking_buffers_[i]->getObservations(marking_observations);
		    current = marking_buffers_[i]->isCurrent() && current;
		    marking_buffers_[i]->unlock();
		}
		marking_observations.insert(marking_observations.end(),
		                              static_marking_observations_.begin(), static_marking_observations_.end());
		return current;
	}

	bool DynamicObjects::getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const
  	{
    	bool current = true;

	    //get the clearing observations
	    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
	    {
	      clearing_buffers_[i]->lock();
	      clearing_buffers_[i]->getObservations(clearing_observations);
	      current = clearing_buffers_[i]->isCurrent() && current;
	      clearing_buffers_[i]->unlock();
	    }
    
    	clearing_observations.insert(clearing_observations.end(), static_clearing_observations_.begin(), static_clearing_observations_.end());

    	return current;
  	}

	void DynamicObjects::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
		const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
    {
    	// filter zeros
	    // Filter positive infinities ("Inf"s) to max_range.
	    float epsilon = 0.0001; // a tenth of a millimeter

	    sensor_msgs::LaserScan message = *raw_message;
	  
	    for( int i = 0; i < message.ranges.size(); i++ )
	    {
	      float range = message.ranges[ i ];
	      if( range >= message.range_max || range <= message.range_min || range == 0.0)
	      {
	        message.ranges[ i ] = message.range_max - epsilon;
	      }
	    }


	    //project the laser into a point cloud
	    sensor_msgs::PointCloud2 cloud;
	    cloud.header = message.header;
	    //project the scan into a point cloud
	    try
	    {
	      projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
	    }
	    catch (tf::TransformException &ex)
	    {
	      ROS_WARN ("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str (), ex.what ());
	      projector_.projectLaser(message, cloud);
	    }
	    //buffer the point cloud
	    buffer->lock();
	    buffer->bufferCloud(cloud);
	    buffer->unlock();

    }

    void DynamicObjects::raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y)
    {

        double ox = clearing_observation.origin_.x;
	    double oy = clearing_observation.origin_.y;
	    pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);

	    //get the map coordinates of the origin of the sensor
	    unsigned int x0, y0;
	    if (!worldToMap(ox, oy, x0, y0))
	    {
	      ROS_WARN_THROTTLE(1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", ox, oy);
	      return;
	    }

	    //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
	    double origin_x = origin_x_, origin_y = origin_y_;
	    double map_end_x = origin_x + size_x_ * resolution_;
	    double map_end_y = origin_y + size_y_ * resolution_;
       
	    touch(ox, oy, min_x, min_y, max_x, max_y);

	    //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
	    for (unsigned int i = 0; i < cloud.points.size(); ++i)
	    {
		    double wx = cloud.points[i].x;
		    double wy = cloud.points[i].y;

            //now we also need to make sure that the enpoint we're raytracing to isn't off the costmap and scale if necessary
      		double a = wx - ox;
      		double b = wy - oy;

      		//the minimum value to raytrace from is the origin
      		if (wx < origin_x)
      		{
        		double t = (origin_x - ox) / a;
        		wx = origin_x;
        		wy = oy + b * t;
      		}

		    if (wy < origin_y)
		    {
		        double t = (origin_y - oy) / b;
		        wx = ox + a * t;
		        wy = origin_y;
		    }

	      	//the maximum value to raytrace to is the end of the map
	      	if (wx > map_end_x)
	      	{
	        	double t = (map_end_x - ox) / a;
	        	wx = map_end_x - .001;
	        	wy = oy + b * t;
	      	}
      
      		if (wy > map_end_y)
      		{
        		double t = (map_end_y - oy) / b;
       			wx = ox + a * t;
        		wy = map_end_y - .001;
      		}

      		//now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      		unsigned int x1, y1;

      		//check for legality just in case
      		if (!worldToMap(wx, wy, x1, y1))
        		continue;

      

      		unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
      		MarkCell marker(costmap_, FREE_SPACE);
      
      		raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
      		updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    	}

    }

    void DynamicObjects::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
		double* max_x, double* max_y)
  	{
	    double dx = wx-ox, dy = wy-oy;
	    double full_distance = hypot(dx, dy);
	    double scale = std::min(1.0, range / full_distance);
	    double ex = ox + dx * scale, ey = oy + dy * scale;
	    touch(ex, ey, min_x, min_y, max_x, max_y);
  	}

  	void DynamicObjects::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) 
  	{
    	//updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    	//updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        

  	}

  	bool DynamicObjects::inObjectBoundingBox(unsigned int x, unsigned int y, std::list<semantic_map::Object>& object_list_)
	{ 
	  	unsigned int mx, my;
	    inside_ = false;
	    
	    std::list<semantic_map::Object>::iterator obs_it;
        for (obs_it = object_list_.begin(); obs_it != object_list_.end(); ++obs_it)
        {
        	semantic_map::Object& object = *obs_it;

		    for (l_ = 0; l_ < 4; l_++) 
		    {
		    	double wx = object.geometry.bounding_box.vertices[l_].x;
		        double wy = object.geometry.bounding_box.vertices[l_].y;

		        layer_grid_->worldToMap(wx, wy, mx, my);

		        polyX[l_] = mx;
		        polyY[l_] = my;

		        //ROS_INFO_STREAM(polyX[l_]);
		        //ROS_INFO_STREAM(polyY[l_]);


		    }
		    //ROS_INFO_STREAM("------");

		/*    polyX[0] = 0;
		    polyY[0] = 0;
		        
		    polyX[1] = 0;
		    polyY[1] = 280;

		    polyX[2] = 280;
		    polyY[2] = 280;

		    polyX[3] = 280;
		    polyY[3] = 0;   */ 
		    

	    
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

	    }
	    return inside_;
	}

	

}; 
