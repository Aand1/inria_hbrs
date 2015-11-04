cmake_minimum_required(VERSION 2.8.3)
project(move_object_actions)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  actionlib_msgs
  tf
  actionlib
  message_generation
  costmap_2d 
  global_planner
  semantic_knowledgebase
)

add_message_files(
   DIRECTORY msg
   FILES
    PushObjectGoals.msg 
    Goal.msg
)

add_action_files(
  DIRECTORY action
  FILES
    Push.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    semantic_knowledgebase
)

catkin_package(
 CATKIN_DEPENDS roscpp semantic_knowledgebase message_runtime std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(push_action src/push_action.cpp)
target_link_libraries(push_action
  ${catkin_LIBRARIES} 
)

add_executable(push_action_test_node
  src/push_action_test_node.cpp
)
target_link_libraries(push_action_test_node push_action)

add_executable(compute_goal_node src/compute_goal_node.cpp)
target_link_libraries(compute_goal_node ${catkin_LIBRARIES})




