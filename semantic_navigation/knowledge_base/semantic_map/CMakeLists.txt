cmake_minimum_required(VERSION 2.8.3)
project(semantic_map)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED
  COMPONENTS 
    cmake_modules
    geometry_msgs
    roscpp
    rospy
    std_msgs
    gazebo_msgs
    yocs_msgs
    message_generation
)


catkin_python_setup()

add_message_files(
   DIRECTORY msg
   FILES
     ObjectInstance.msg
     Object.msg
     ObjectList.msg

     RegionInstance.msg
     Region.msg
     RegionList.msg

     SemanticMapMessage.msg
     
     GeometricProperties.msg
     SemanticProperties.msg
     BoundingBox.msg 

     SemanticPositions.msg
)

add_service_files(
   FILES
     QueryRegionInstances.srv
     SemanticMapQueryResponse.srv
   
 )
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
     geometry_msgs
     std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      query_regions
      semantic_map_query
      semantic_map
    CATKIN_DEPENDS 
      message_runtime 
      std_msgs
      geometry_msgs
      gazebo_msgs
      yocs_msgs
    DEPENDS
      Boost
      #yaml-cpp
      gazebo
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 #${yaml-cpp_INCLUDE_DIRS}
)

add_library(query_regions
  src/query_regions.cpp
)
add_library(semantic_map_query
  src/semantic_map_query.cpp
)
add_library(semantic_map
  src/semantic_map.cpp
)

add_dependencies(query_regions
  semantic_map_generate_messages_cpp
)
add_dependencies(semantic_map_query
  semantic_map_generate_messages_cpp
)
add_dependencies(semantic_map
  semantic_map_generate_messages_cpp
)

target_link_libraries(query_regions
  ${catkin_LIBRARIES}
)
target_link_libraries(semantic_map_query
  ${catkin_LIBRARIES}
)
target_link_libraries(semantic_map
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

install(TARGETS query_regions semantic_map_query semantic_map
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


