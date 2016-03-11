cmake_minimum_required(VERSION 2.8.3)
project(semantic_planner_global)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    sem_nav_msgs
    global_planner
    semantic_map
    costmap_2d
    semantic_costmap
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${semantic_costmap_INCLUDE_DIRS}
)

add_library(semantic_planner_global 
  src/semantic_dijkstra.cpp
  src/planner_core.cpp
  src/semantic_planner_global.cpp
  
  
)
target_link_libraries(semantic_planner_global 
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${semantic_costmap_LIBRARIES}
  semantic_map 
)

add_executable(semantic_planner_global_node
  src/semantic_planner_global_node.cpp
)
target_link_libraries(semantic_planner_global_node ${catkin_LIBRARIES} semantic_planner_global)



