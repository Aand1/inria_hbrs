/*#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h> 
#include <semantic_map/Object.h> 
#include <semantic_map/semantic_map.h>

#include <iostream>
#include <cmath>
#include <math.h> 
#include <list>
#include <vector>
#include <iostream>

using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "costmap_test_node");	
	tf::TransformListener tf(ros::Duration(10));
	ros::NodeHandle nh_;

	costmap_2d::Costmap2DROS* global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf);
	semantic_map::SemanticMap* semantic_map_query = new semantic_map::SemanticMap(nh_);

	std::list<semantic_map::Object> object_list = semantic_map_query->getSemanticMap(); 

	double* min_x = new double;
	*min_x = 1000.0;
	double* min_y = new double;
	*min_y = 1000.0;
	double* max_x = new double;
	*max_x = 0.0;
	double* max_y = new double;
	*max_y = 0.0;


	list<semantic_map::Object>::iterator obs_it;
    for (obs_it = object_list.begin(); obs_it != object_list.end(); ++obs_it)
    {
    	semantic_map::Object& object = *obs_it;

    	ROS_INFO_STREAM(object);

    	for (int i = 0; i<4; i++)
    	{
    		double wx = object.geometry.bounding_box.vertices[i].x;
    		double wy = object.geometry.bounding_box.vertices[i].y;

    		unsigned int mx, my, omx, omy;

    		global_costmap_->getCostmap()->worldToMap(wx, wy, mx, my);

    		double center_x = object.geometry.pose.position.x;
    		double center_y = object.geometry.pose.position.y;
    		global_costmap_->getCostmap()->worldToMap(center_x, center_y, omx, omy);

    		double res = global_costmap_->getCostmap()->getResolution();
    		double factor = 1.0;

    		int dx = mx - omx;
    		int dy = my - omy;
    		//ROS_INFO_STREAM(dx);
    		//ROS_INFO_STREAM(dy);

    		double h = sqrt(dx*dx+dy*dy) + factor/res;
    		double angle = atan2(dy,dx);

    		double x = mx + cos(angle) * h;
    		double y = my + sin(angle) * h;

    		ROS_INFO_STREAM(x);
    		ROS_INFO_STREAM(y);
    		ROS_INFO_STREAM("-------------");

    		*min_x = std::min(*min_x, x);
            *min_y = std::min(*min_y, y);
            *max_x = std::max(*max_x, x);
            *max_y = std::max(*max_y, y);

    	}
    	ROS_INFO_STREAM(*min_x);
    	ROS_INFO_STREAM(*min_y);
    	ROS_INFO_STREAM(*max_x);
    	ROS_INFO_STREAM(*max_y);
    	ROS_INFO_STREAM("+++++++++++++++++");

    	unsigned char cost;

    	while (nh_.ok())
    	{

	    	for (int x = *min_x; x < *max_x; x++)
	    	{
	    		for (int y = *min_y; y < *max_y; y++)
	    		{
	    			//cost = global_costmap_->getCostmap()->getCost(x, y);
	    			//ROS_INFO_STREAM(cost);
	    			//std::cout << cost << std::endl;
	    			//if (cost >=  costmap_2d::FREE_SPACE && cost <  250)
	    			//	ROS_INFO_STREAM("hi");
	    			//global_costmap_->getCostmap()->setCost(x, y, costmap_2d::LETHAL_OBSTACLE);

	    		}
	    	}
	    }



    }
    	


	
	//ros::MultiThreadedSpinner s;
    ros::spin();

	return 0;

}*/

#include <ros/ros.h>
#include <semantic_costmap/semantic_costmap_ros.h>
#include "std_msgs/Int32.h" 
#include <sem_nav_msgs/SemanticCostmapConstraints.h>    

semantic_costmap::SemanticCostmapROS* global_costmap_;

/*void constraintsCallback(const sem_nav_msgs::SemanticCostmapConstraints::ConstPtr& constraints)
{
    //ROS_INFO_STREAM("Semantic costmap node");

    sem_nav_msgs::SemanticCostmapConstraints constraints_;

    constraints_ = *(constraints);

    global_costmap_->setParameters(constraints_);

}*/

void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
   if (msg->data == 0) 
   {
       ROS_INFO_STREAM("Stop costmap");
       //global_costmap_->stop();
       //global_costmap_->startLayers();
       //global_costmap_->start();
       global_costmap_->stopCostmap();

    }

    else if (msg->data == 1) 
    {
       ROS_INFO_STREAM("Starting costmap");
       //global_costmap_->stopLayers();
       //global_costmap_->startLayers();
    }

    else if (msg->data == 3) 
    {
       ROS_INFO_STREAM("Starting costmap normally");
       //global_costmap_->stopLayers();
       global_costmap_->start();
    }

     
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "semantic_costmap_node");	
	tf::TransformListener tf(ros::Duration(10));
    ros::NodeHandle nh_;
    

    ros::Subscriber sub = nh_.subscribe("semantic_costmap_node", 1000, chatterCallback);
    //ros::Subscriber sem_cm_sub = nh_.subscribe("semantic_costmap_constraints", 0, constraintsCallback);    

	global_costmap_ = new semantic_costmap::SemanticCostmapROS("global_costmap", tf);
    //global_costmap_->start();
    //global_costmap_->stop();
    //global_costmap_->stopLayers();
    //ros::Duration(1.0).sleep();
    //global_costmap_->start();

    //global_costmap_->stopLayers();
    // make sure to sleep for the remainder of our cycle time
    
    //global_costmap_->stopLayers();
    //global_costmap_->stop();
    //global_costmap_->stopLayers();
	//global_costmap_->resetLayers();
        //global_costmap_->start();
   
    //ros::Duration(5.0).sleep(); 
        //while (nh_.ok())
	//{
    //global_costmap_->startLayers();
	//}

    //costmap_2d::Costmap2DROS* global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf);
    

    
    

	ros::spin();

	return 0;

}

