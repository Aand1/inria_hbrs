cmake_minimum_required(VERSION 2.8.3)
project(semantic_navigation_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  sem_nav_msgs
  semantic_map
)

find_package(semantic_planner_global REQUIRED)


find_package(Boost REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES semantic_navigation_planner
  CATKIN_DEPENDS roscpp tf std_msgs geometry_msgs nav_msgs sem_nav_msgs 
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${roscpp_INCLUDE_DIRS}
  ${semantic_planner_global_INCLUDE_DIRS}
  ${sem_nav_msgs_INCLUDE_DIRS} 
)

add_library(semantic_planner
 src/compute_best_path.cpp
 src/reasoner.cpp
 src/semantic_planner.cpp
)
target_link_libraries(semantic_planner
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${semantic_planner_global_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
  semantic_map
)

#add_executable(compute_best_path_node src/compute_best_path_node.cpp)
#target_link_libraries(compute_best_path_node ${catkin_LIBRARIES} semantic_planner)

#add_executable(constraints_node src/constraints_node.cpp)
#target_link_libraries(constraints_node ${catkin_LIBRARIES} semantic_planner)

add_executable(semantic_planner_node src/semantic_planner_node.cpp)
target_link_libraries(semantic_planner_node ${catkin_LIBRARIES} semantic_planner)
