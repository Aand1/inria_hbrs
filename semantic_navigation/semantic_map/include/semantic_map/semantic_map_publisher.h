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

#ifndef SEMANTIC_MAP_PUBLISHER_H_
#define SEMANTIC_MAP_PUBLISHER_H_

#include <ros/ros.h>
#include <semantic_map/semantic_map_query.h>


namespace semantic_map
{
    /**
     * @class SemanticMapPublisher
     * @brief A class to periodically publish information about objects and regions in the environment
     */
    class SemanticMapPublisher
    {
    public:
    	/**
         * @brief  Constructor for the SemanticMapPublisher
         */
        SemanticMapPublisher(ros::NodeHandle* nh);
        
        /**
         * @brief  Destructor
         */
        ~SemanticMapPublisher(); 

        /**
         * @brief  Publisher for sending semantic map information to various other components
         */
        void publishSemanticMap();

        /**
         * @brief Checker to see if the publisher is active
         * @return True if the frequency for the publisher is non-zero, false otherwise
         */
        bool active()
        {
            return active_;
        }

    private:
    	/**
    	 * @brief  Method convert information in semantic map into a ROS message
    	 */
    	semantic_map::SemanticMapMessage& prepareMessage(); 

    	void querySemanticMap();

    	/** @brief Publisher to publish the latest semantic map to the new subscriber. */
        void onNewSubscription(const ros::SingleSubscriberPublisher& pub);

        semantic_map::SemanticMapQuery* semantic_map_query;
        semantic_map::SemanticMapMessage semantic_map_message;

        ros::NodeHandle* nh_;
        bool active_;
        ros::Publisher semantic_map_pub_;

    };


}
#endif

