cmake_minimum_required(VERSION 2.8.3)
project(semantic_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sem_nav_msgs
  tf
  message_generation
  semantic_map
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs tf 
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${semantic_map_INCLUDE_DIRS}
)

add_library(semantic_localization
   src/semantic_localization.cpp
)

target_link_libraries(semantic_localization
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
  semantic_map
)

add_executable(semantic_localization_node
  src/semantic_localization_node.cpp
)
target_link_libraries(semantic_localization_node semantic_localization)


