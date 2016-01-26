#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher path_markers_pub = n.advertise<visualization_msgs::MarkerArray>("path_markers", 10);
 
  ros::Rate r(30);
 
  //float f = 0.0;

  
    visualization_msgs::MarkerArray path_markers; 
    path_markers.markers.resize(2);
    visualization_msgs::Marker path_marker1, path_marker2;
    
    ////////////////////////////////////////////////////////////////////////
    path_marker1.header.frame_id = "/my_frame";
    path_marker1.header.stamp = ros::Time::now();
    path_marker1.id = 0;
    path_marker1.ns = "points_and_lines";
    path_marker1.header.stamp = ros::Time::now();
    path_marker1.ns = "points_and_lines";
    path_marker1.action = visualization_msgs::Marker::ADD;
    path_marker1.type = visualization_msgs::Marker::POINTS;
    path_marker1.scale.x = 0.1;
    path_marker1.scale.y = 0.1;
    path_marker1.scale.z = 0.1;
    path_marker1.color.r = 1.0f;
    path_marker1.color.g = 0.0f;
    path_marker1.color.b = 0.0f;
    path_marker1.color.a = 1.0;
    path_marker1.lifetime = ros::Duration(20.0);
    path_marker1.pose.orientation.w = 1.0;
    path_markers.markers.push_back(path_marker1);

    ////////////////////////////////////////////////////////////////////////
    path_marker2.header.frame_id = "/my_frame";
    path_marker2.header.stamp = ros::Time::now();
    path_marker2.id = 1;
    //path_marker2.ns = "points_and_lines";
    path_marker2.header.stamp = ros::Time::now();
    path_marker2.ns = "points_and_lines";
    path_marker2.action = visualization_msgs::Marker::ADD;
    path_marker2.type = visualization_msgs::Marker::POINTS;
    path_marker2.scale.x = 0.1;
    path_marker2.scale.y = 0.1;
    path_marker2.scale.z = 0.1;
    path_marker2.color.r = 0.0f;
    path_marker2.color.g = 1.0f;
    path_marker2.color.b = 0.0f;
    path_marker2.color.a = 1.0;
    path_marker2.pose.orientation.w = 1.0;
    path_markers.markers.push_back(path_marker2);

     
    // Create the vertices for path_marker1
/*    for (float i = 0; i < 100; i++)
    {
      
      geometry_msgs::Point p;
      p.x = i/8;//(int32_t)i - 50;
      p.y = 0.0;
      p.z = 0.0;
      path_marker1.points.push_back(p);
      path_markers.markers.push_back(path_marker1);*/

	/* /*   path_marker1.header.frame_id = "/map";
	    //path_marker1.header.stamp = ros::Time::now();
	    //path_marker1.id = 0;
	    //path_marker1.ns = "points_and_lines";
	    path_marker1.header.stamp = ros::Time::now();
	    path_marker1.ns = "points_and_lines";
	    //path_marker1.action = visualization_msgs::Marker::SPHERE;
	    path_marker1.type = visualization_msgs::Marker::POINTS;
	    path_marker1.scale.x = 1.1;
	    path_marker1.scale.y = 1.1;
	    path_marker1.scale.z = 1.1;
	    path_marker1.color.r = 1.0f;
	    path_marker1.color.g = 0.0f;
	    path_marker1.color.b = 0.0f;
	    path_marker1.color.a = 1.0;
	    path_marker1.lifetime = ros::Duration(10.0);
	    //path_marker1.pose.orientation.w = 1.0;
	  
	      
	    path_marker1.pose.position.x = (int32_t)i - 50;
	    //path_marker1.pose.position.z = 0.60;
	    path_markers.markers.push_back(path_marker1);
	      //ROS_INFO_STREAM(path_markers);*/

    //}*/

    // Create the vertices for path_marker2
/*    for (float i = 0; i < 100; i++)
    {
      
      geometry_msgs::Point p;
      p.x = 0.0;//(int32_t)i - 50;
      p.y = i/8;
      p.z = 0.0;
  
      path_marker2.points.push_back(p);
    }*/

    for (float i = 0; i < 100; i++)
    {
      
      geometry_msgs::Point p;
      p.x = i/8;//(int32_t)i - 50;
      p.y = 0.0;
      p.z = 0.0;
      path_marker1.points.push_back(p);
      //path_markers.markers.push_back(path_marker1);

      
      
    } 

    for (float i = 0; i < 100; i++)
    {
      
      geometry_msgs::Point p;
      p.x = i/8;//(int32_t)i - 50;
      p.y = i/8;
      p.z = 0.0;
      path_marker2.points.push_back(p);
      //path_markers.markers.push_back(path_marker2);

      
      
    } 

    path_markers.markers.push_back(path_marker1);
    path_markers.markers.push_back(path_marker2);

 while (ros::ok())
  {
    //marker_pub.publish(path_marker1);
    path_markers_pub.publish(path_markers);
    
  
    r.sleep();
  
    
  }
}