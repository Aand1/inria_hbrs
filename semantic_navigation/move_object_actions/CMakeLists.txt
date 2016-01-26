cmake_minimum_required(VERSION 2.8.3)
project(move_object_actions)
find_package(catkin REQUIRED 
  COMPONENTS
    cmake_modules
    roscpp
    actionlib
    std_msgs
    nav_msgs
    geometry_msgs
    semantic_map
    semantic_costmap
    dynamic_reconfigure
    
    #sem_nav_msgs
  
)

find_package(semantic_planner_global REQUIRED)
find_package(semantic_planner_local REQUIRED)
find_package(goal_monitor REQUIRED)

catkin_package(
 CATKIN_DEPENDS 
   roscpp
   dynamic_reconfigure
        
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_LIBRARIES} 
  ${semantic_planner_global_INCLUDE_DIRS}
  ${semantic_planner_local_INCLUDE_DIRS}
  ${goal_monitor_INCLUDE_DIRS}
)

add_library(push_action src/push_action.cpp)
target_link_libraries(push_action
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${semantic_planner_global_LIBRARIES}
  ${semantic_planner_local_LIBRARIES}
  ${goal_monitor_LIBRARIES}
  semantic_map
  semantic_costmap
)


add_executable(push_action_node
  src/push_action_node.cpp
)
target_link_libraries(push_action_node ${catkin_LIBRARIES} push_action)






