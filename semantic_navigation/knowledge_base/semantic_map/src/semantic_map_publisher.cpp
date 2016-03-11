/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/

 #include <semantic_map/semantic_map_publisher.h>

 namespace semantic_map
 {

    SemanticMapPublisher::SemanticMapPublisher(ros::NodeHandle* nh) :
        nh_(nh),
        active_(false)
    {
    	semantic_map_pub_ = nh_->advertise<semantic_map::SemanticMapMessage>("semantic_map_message", 1, true);
    	semantic_map_query = new semantic_map::SemanticMapQuery(*nh_);

    	publishSemanticMap();
    	
    }

    SemanticMapPublisher::~SemanticMapPublisher()
    {
    	
    }

    void SemanticMapPublisher::publishSemanticMap()
    {
    	
    	//if (semantic_map_pub_.getNumSubscribers() > 0)
        {
          semantic_map_message = prepareMessage();
          //prepareMessage();
          semantic_map_pub_.publish(semantic_map_message);

          ROS_INFO_STREAM("Semantic map message published");
        }
    	
    }

    semantic_map::SemanticMapMessage& SemanticMapPublisher::prepareMessage()
    {
    	//querySemanticMap();
        SemanticMapMessage& message = semantic_map_query->query();
        //*message = semantic_map_query->query();
    	return message;
    }

    void SemanticMapPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
    {

    }

    void SemanticMapPublisher::querySemanticMap()
    {
        //region_list_ = query_regions->query();
        //SemanticMapMessage* message = semantic_map_query->query();

        //ROS_INFO_STREAM(region_list_);

    }



 }  // end namespace semantic_map