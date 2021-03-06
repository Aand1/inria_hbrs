cmake_minimum_required(VERSION 2.8.3)
project(semantic_map)

find_package(catkin REQUIRED COMPONENTS 
   cmake_modules
   geometry_msgs
   roscpp
   rospy
   std_msgs
   message_generation
  
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

catkin_python_setup()

add_message_files(
   DIRECTORY msg
   FILES
     ObjectInstance.msg
     Object.msg
     ObjectList.msg

     RegionInstance.msg
     Region.msg
     RegionList.msg

     SemanticMapMessage.msg
     
     GeometricProperties.msg
     SemanticProperties.msg
     BoundingBox.msg 

     SemanticPositions.msg
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
add_service_files(
   FILES
     QueryRegionInstances.srv
     SemanticMapQueryResponse.srv
   
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
     geometry_msgs
     std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES query_regions
    CATKIN_DEPENDS 
      roscpp 
      message_runtime 
      roscpp 
      rospy 
      std_msgs
      geometry_msgs
    DEPENDS
      Boost
      yaml-cpp
      gazebo_msgs
      
)

include_directories(include)
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${roscpp_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${yaml-cpp_INCLUDE_DIRS}
)

add_library(query_regions
  src/query_regions.cpp
)
target_link_libraries(query_regions
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
)

add_library(semantic_map_query
  src/semantic_map_query.cpp
)
target_link_libraries(semantic_map_query
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
)




add_library(semantic_map
  src/semantic_map.cpp
)
target_link_libraries(semantic_map
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
  #yamlcpp_convert
)

#add_executable(semantic_map_node src/semantic_map_node.cpp)
#target_link_libraries(semantic_map_node ${catkin_LIBRARIES} semantic_map)



#add_executable(query_test_node src/query_test_node.cpp)
#target_link_libraries(query_test_node ${catkin_LIBRARIES} query_regions semantic_map_query)

#add_library(semantic_map_publisher src/semantic_map_publisher.cpp)
#target_link_libraries(semantic_map_publisher ${catkin_LIBRARIES} semantic_map_query)

#add_executable(semantic_map_publisher_node src/semantic_map_publisher_node.cpp)
#target_link_libraries(semantic_map_publisher_node semantic_map_publisher)

#add_executable(semantic_map_query_node src/semantic_map_query_test.cpp)
#target_link_libraries(semantic_map_query_node semantic_map)





