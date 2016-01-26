cmake_minimum_required(VERSION 2.8.3)
project(semantic_planner_global)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    sem_nav_msgs
    pluginlib
    roscpp
    tf
    global_planner
    semantic_map
    semantic_costmap
    
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    sem_nav_msgs
    pluginlib
    roscpp
    tf
    global_planner
    
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${semantic_costmap_INCLUDE_DIRS}
)

add_library(planner_global
  src/planner_core.cpp
)
target_link_libraries(planner_global
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  global_planner
)

add_library(semantic_planner_global 
  src/semantic_planner_global.cpp
  src/semantic_dijkstra.cpp
)
target_link_libraries(semantic_planner_global 
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${semantic_costmap_LIBRARIES}
  planner_global
  semantic_map
  global_planner
 
)

add_executable(semantic_planner_global_node
  src/semantic_planner_global_node.cpp
)
target_link_libraries(semantic_planner_global_node ${catkin_LIBRARIES} semantic_planner_global)






