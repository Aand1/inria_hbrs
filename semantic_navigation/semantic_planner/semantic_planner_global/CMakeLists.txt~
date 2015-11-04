cmake_minimum_required(VERSION 2.8.3)
project(semantic_planner_global)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_core
  pluginlib
  roscpp
  tf
  global_planner
  dynamic_reconfigure
  
)

generate_dynamic_reconfigure_options(
  cfg/SemanticPlannerGlobal.cfg
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES global_planner
  CATKIN_DEPENDS base_local_planner costmap_2d nav_core roscpp tf dynamic_reconfigure std_msgs global_planner
  #DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(semantic_planner_global 
  src/semantic_planner_global.cpp
  src/quadratic_calculator.cpp
  src/astar.cpp
  src/dijkstra.cpp
  src/gradient_path.cpp
  src/grid_path.cpp
)
target_link_libraries(semantic_planner_global 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
)
add_dependencies(semantic_planner_global semantic_planner_global_gencfg)

add_executable(semantic_planner_global_node
  src/semantic_planner_global_node.cpp
)
target_link_libraries(semantic_planner_global_node semantic_planner_global)
set_target_properties(semantic_planner_global_node PROPERTIES OUTPUT_NAME semantic_planner_global)




