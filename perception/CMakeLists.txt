cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  semantic_map
  message_generation
)


add_message_files(
   DIRECTORY msg
   FILES
     ObjectState.msg
     ObjectsState.msg
     
)

generate_messages(
   DEPENDENCIES
     geometry_msgs
     std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    dummy_perception
    gazebo_ros_objects_state_publisher
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    gazebo_msgs
    message_runtime
    semantic_map
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(dummy_perception
  src/dummy_perception.cpp
)

add_dependencies(dummy_perception
  peception_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(dummy_perception
  ${catkin_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_LIBRARIES}
  #semantic_map
)

add_library(gazebo_ros_objects_state_publisher plugins/gazebo_ros_objects_state_publisher.cpp)
add_dependencies(gazebo_ros_objects_state_publisher
  peception_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


target_link_libraries(gazebo_ros_objects_state_publisher ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(dummy_perception_node src/dummy_perception_node.cpp)
add_dependencies(dummy_perception_node
  peception_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(dummy_perception_node ${catkin_LIBRARIES} dummy_perception )

#add_executable(yaml_test src/test_1.cpp)
#target_link_libraries(yaml_test ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

#add_executable(read_yaml src/read_yaml.cpp)
#target_link_libraries(read_yaml ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})
#add_executable(write_yaml src/write_yaml.cpp)
#target_link_libraries(write_yaml ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} semantic_map)


##add_library(hello_world SHARED plugins/hello_world.cc)
#target_link_libraries(hello_world ${GAZEBO_LIBRARIES})





