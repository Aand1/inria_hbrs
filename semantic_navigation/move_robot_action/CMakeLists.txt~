cmake_minimum_required(VERSION 2.8.3)
project(move_robot_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  sem_nav_msgs
  actionlib
  costmap_2d
  semantic_costmap
  nav_core

)
find_package(semantic_planner_global REQUIRED)
find_package(semantic_planner_local REQUIRED)
find_package(goal_monitor REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_robot_action
  CATKIN_DEPENDS roscpp dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_LIBRARIES}
  ${semantic_planner_global_INCLUDE_DIRS}
  ${semantic_planner_local_INCLUDE_DIRS}
  ${goal_monitor_INCLUDE_DIRS}
)

add_library(move_robot_action 
  src/move_robot_action.cpp)

target_link_libraries(move_robot_action 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${semantic_planner_global_LIBRARIES}
  ${semantic_planner_local_LIBRARIES}
  ${goal_monitor_LIBRARIES}
  semantic_costmap
)

add_executable(move_robot_action_node
  src/move_robot_action_node.cpp
)
target_link_libraries(move_robot_action_node 
  ${catkin_LIBRARIES}
  move_robot_action
)


