cmake_minimum_required(VERSION 2.8.3)
project(semantic_costmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  costmap_2d
  semantic_map
  pluginlib
  dynamic_reconfigure
  
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  config/StructuralObjectsPlugin.cfg
  config/HeavyObjectsPlugin.cfg
  config/LightObjectsPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES semantic_costmap 
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
  DEPENDS
    Eigen
    Boost
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(semantic_costmap
  src/static_objects.cpp
  src/dynamic_objects.cpp
  src/inflate_objects.cpp
  src/semantic_costmap_ros.cpp
)

target_link_libraries(semantic_costmap
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  semantic_map
  
)

add_library(semantic_layers 
   plugins/structural_objects_layer.cpp
   plugins/heavy_objects_layer.cpp
   plugins/light_objects_layer.cpp
)
target_link_libraries(semantic_layers
   ${catkin_LIBRARIES}
   ${BOOST_LIBRARIES}
   semantic_costmap

)



add_executable(semantic_costmap_node
  src/semantic_costmap_node.cpp
)
target_link_libraries(semantic_costmap_node ${catkin_LIBRARIES} ${roscpp_LIBRARIES} semantic_costmap)










