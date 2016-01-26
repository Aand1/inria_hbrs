cmake_minimum_required(VERSION 2.8.3)
project(sem_nav_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  semantic_map  
  dynamic_reconfigure
  message_generation

)


add_message_files(
   DIRECTORY msg
   FILES
    SemanticPose.msg 
    MoveObjectGoals.msg
    MoveObjectPaths.msg

    GlobalPlannerConstraints.msg
    LocalPlannerConstraints.msg
    LayerConstraints.msg
    SemanticCostmapConstraints.msg
    Constraints.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPlanObject.srv  
)

add_action_files(
  DIRECTORY action
  FILES
    Push.action
    MoveRobot.action
)

generate_messages(
  DEPENDENCIES 
    geometry_msgs 
    std_msgs 
    nav_msgs 
    semantic_map
    dynamic_reconfigure
    actionlib_msgs    

)
  
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES viz_msgs
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs semantic_map
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(viz_msgs
 src/viz_msgs.cpp
)
target_link_libraries(viz_msgs ${catkin_LIBRARIES} ${Boost_LIBRARIES})


