#include <ros/ros.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>


int main()
{
    YAML::Node node = YAML::LoadFile("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test1.yaml");

    if (node.IsSequence()) 
    {
      std::cout << "Is Sequence\n" ; // prints "Hello, World!"
    }

    if (node.IsNull()) 
    {
      std::cout << "Is Null\n" ;
    }

    if (node.IsScalar()) 
    {
      std::cout << "Is Scalar\n" ;
    }

    if (node.IsMap()) 
    {
      std::cout << "Is Map\n" ;
    }

    double value = node[0]["position"]["x"].as<double>();

    std::cout << value <<"\n";


}
