#include <ros/ros.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct Vec3 { int x; int y; int z; };

struct Position { int x; int y; int z; };
struct Orientation { int x; int y; int z; int w; };

struct Pose
{
    Position p;
    Orientation o;

};

namespace YAML {
template<>
struct convert<Vec3> {
  static Node encode(const Vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, Vec3& rhs) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};
}

// now the extraction operators for these types
void operator << (YAML::Emitter& out, Pose& v) {
    out << YAML::BeginMap;
    out << YAML::Key << "position";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;
}

int main()
{
/*   YAML::Node node = YAML::Load("start: [1, 3, 0]");
    //Vec3 v = node[0].as<Vec3>();
    
    Vec3 v1;
    v1.x = 2;
    v1.y = 4;
    v1.z = 6;
    node[0] = v1;//Vec3(2, -1, 0);

    Vec3 v2;
    v1.x = 3;
    v1.y = 6;
    v1.z = 9;
    node[1] = v2;

    std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test.yaml");
    fout << node;

//    YAML::Node node = YAML::Load("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test.yaml");
    
//    node["instance"].as<std::string>();
    


    YAML::Node node;  // starts out as null
 
    node[0]["instance"] = "box-1";
    node[0]["geometry"]["pose"]["position"]["x"] = 1.4;
    node[0]["geometry"]["pose"]["position"]["y"] = 1.5;
    node[0]["geometry"]["pose"]["position"]["z"] = 1.6;

    node[0]["geometry"]["pose"]["orientation"]["x"] = 1.1;
    node[0]["geometry"]["pose"]["orientation"]["y"] = 1.2;
    node[0]["geometry"]["pose"]["orientation"]["z"] = 1.3;
    node[0]["geometry"]["pose"]["orientation"]["w"] = 1.3;

    node[0]["semantics"]["category"] = "structural_object";
    node[0]["semantics"]["sub_category"] = "Walls";

    node[1]["instance"] = "box-1";
    node[1]["geometry"]["pose"]["position"] = "";
    

    std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test.yaml");
    fout << node;*/

//    std::string key = node[0]["instance"].as<std::string>();
//    std::cout << key << '\n';
    




    YAML::Emitter out;
//    Pose p;
    //out << p;
    
/*
    out << YAML::BeginSeq;
    out << YAML::BeginMap;
   
    out << YAML::Key << "instance";
    out << YAML::Value << "";

    out << YAML::Key << "geometry";
    ///////////////////////////////////////////////
    out << YAML::BeginMap;
    out << YAML::Key << "pose";

    out << YAML::BeginMap;
    out << YAML::Key << "position";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;

    out << YAML::Key << "orientation";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::Value << 0.0;
    out << YAML::Key << "w";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;

    out << YAML::EndMap;

    out << YAML::Key << "bounding_box";
    out << YAML::BeginMap;
    out << YAML::Key << "vertices";
    out << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;
   
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;

    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;

    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;

    out << YAML::EndSeq;

    out << YAML::EndMap;

    out << YAML::EndMap;
    
    //////////////////////////////////////////////
    out << YAML::Key << "semantics";
    out << YAML::Value << "";

    out << YAML::EndMap;
    //out << YAML::EndSeq;



    //out << YAML::BeginSeq;
    
    //out << YAML::EndSeq;

    out << YAML::BeginMap;
   
    out << YAML::Key << "instance";
    out << YAML::Value << "";

    out << YAML::Key << "geometry";
    ///////////////////////////////////////////////
    out << YAML::BeginMap;
    out << YAML::Key << "pose";

    out << YAML::BeginMap;
    out << YAML::Key << "position";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;

    out << YAML::Key << "orientation";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::Value << 0.0;
    out << YAML::Key << "w";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;

    out << YAML::EndMap;

    out << YAML::Key << "bounding_box";
    out << YAML::BeginMap;
    out << YAML::Key << "vertices";
    out << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;
   
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key <<
 "z";
    out << YAML::EndMap;

    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;

    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << 0.0;
    out << YAML::Key << "y";
    out << YAML::Value << 0.0;
    out << YAML::Key << "z";
    out << YAML::EndMap;

    out << YAML::EndSeq;

    out << YAML::EndMap;

    out << YAML::EndMap;
    
    //////////////////////////////////////////////
    out << YAML::Key << "semantics";
    out << YAML::Value << "";

    out << YAML::EndMap;


    std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test1.yaml");
    fout << out.c_str();
    //YAML::Node node;  // starts out as null
    //node[]["eggs"] = "value1";
    //node["eggs"] = "value1";

    //node[0] = "fruits";
    //node["fruits"] = "value2";
    
   // std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test.yaml");
   // fout << out;

}

/*
struct Vec3 { int x; int y; int z; };

struct Power {
   std::string name;
   int damage;
};

struct Monster {
   std::string name;
   Vec3 position;
   std::vector <Power> powers;
};

YAML::Emitter& operator <
< (YAML::Emitter& out, const Vec3& v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
    return out;
}*/


/*

int main()
{
    //YAML::Node basenode = YAML::Load("monsters.yaml");
    //string name;
    //doc[1] >> name;
    //name = doc[name].as<string>();
    //const YAML::Node& configuration = basenode["configuration"];
    //int height = basenode["configuration"][0]["height"].as<int>();
    //basenode["basenode"].as<string>();
    YAML::Emitter out;
    std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test1.yaml");
    // list
    out << YAML::BeginSeq;
    out << "eggs";
    out << "bread";
    out << "milk";
    out << YAML::EndSeq;
   // YAML::Node node = test;
    //std::ofstream fout("monsters.yaml");
    fout << out.c_str();

    //YAML::Node node;  // starts out as null
    //YAML::Emitter out;
    //YAML::Emitter out;
    //out << YAML::Flow;
   // out << YAML::BeginSeq << 2 << 3 << 5 << 7 << 11 << YAML::EndSeq;
   // std::ofstream fout("monsters.yaml");
   // node[0] = "eggs";
    //node["key"] = "value";  // it now is a map node
    //std::ofstream fout("/home/niranjan/catkin_ws/src/inria_hbrs/perception/data/test1.yaml");
    //fout << node;

    //YAML::Emitter emitter;
    //emitter << "Hello world!";

    //std::ofstream fout("monsters.yaml");
    //fout << emitter.c_str();




    return 0;
}*/

/*int main()
{
   YAML::Node node = YAML::Load("data/monsters.yaml");

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

   for (YAML::const_iterator it=node.begin();it!=node.end();++it) {
      std::cout << it->as<int>() << "\n";
   }

   node["height"].as<std::string>();


   

   
    return 0;
}*/
