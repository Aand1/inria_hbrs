#include <ros/ros.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>
#include <semantic_map/GeometricProperties.h>
#include <geometry_msgs/Vector3.h>

namespace YAML {
template<>
struct convert<geometry_msgs::Vector3> {
  static Node encode(const geometry_msgs::Vector3& rhs) {
    Node node;

    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);

//    node["pose"]["position"]["x"] = rhs.pose.position.x;

//    node["x"].push_back(rhs.bounding_box.vertices[0].x);
    
//    node["vertex-1"]["x"] = rhs.vertices[0].x;
/*    node["vertex-1"]["y"] = rhs.one.y;
    node["vertex-1"]["z"] = rhs.one.z;

    node["vertex-2"]["x"] = rhs.two.x;
    node["vertex-2"]["y"] = rhs.two.y;
    node["vertex-2"]["z"] = rhs.two.z;

    node["vertex-3"]["x"] = rhs.three.x;
    node["vertex-3"]["y"] = rhs.three.y;
    node["vertex-3"]["z"] = rhs.three.z;

    node["vertex-4"]["x"] = rhs.four.x;
    node["vertex-4"]["y"] = rhs.four.y;
    node["vertex-4"]["z"] = rhs.four.z;*/

    return node;
  }
};
}


namespace YAML {
template<>
struct convert<semantic_map::GeometricProperties> {
  static Node encode(const semantic_map::GeometricProperties& rhs) {
    Node node;

    node["pose"]["position"]["x"] = rhs.pose.position.x;

    node["bounding_box"]["vertex-1"] = rhs.bounding_box.vertices[0];
    node["bounding_box"]["vertex-2"] = rhs.bounding_box.vertices[1];
    node["bounding_box"]["vertex-3"] = rhs.bounding_box.vertices[2];
    node["bounding_box"]["vertex-4"] = rhs.bounding_box.vertices[3];
    
//    node["vertex-1"]["x"] = rhs.vertices[0].x;
/*    node["vertex-1"]["y"] = rhs.one.y;
    node["vertex-1"]["z"] = rhs.one.z;

    node["vertex-2"]["x"] = rhs.two.x;
    node["vertex-2"]["y"] = rhs.two.y;
    node["vertex-2"]["z"] = rhs.two.z;

    node["vertex-3"]["x"] = rhs.three.x;
    node["vertex-3"]["y"] = rhs.three.y;
    node["vertex-3"]["z"] = rhs.three.z;

    node["vertex-4"]["x"] = rhs.four.x;
    node["vertex-4"]["y"] = rhs.four.y;
    node["vertex-4"]["z"] = rhs.four.z;*/

    return node;
  }
};
}

int main()
{
    YAML::Node node;  // starts out as null
    semantic_map::GeometricProperties g1;
   geometry_msgs::Vector3 v;

    

    g1.bounding_box.vertices.reserve(10); 
    g1.bounding_box.vertices[0] = v;
    g1.bounding_box.vertices[1] = v;
    g1.bounding_box.vertices[2] = v;
    g1.bounding_box.vertices[4] = v;

    //ROS_INFO_STREAM(g1.bounding_box.vertices[0]);


    node[0] = g1; 
    std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/write_test.yaml");
    fout << node;



}
