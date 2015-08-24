cmake_minimum_required(VERSION 2.8.3)
project(move_robot)

find_package(catkin REQUIRED 
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        nav_core
        tf
        
)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveRobot.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_robot
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(move_robot_configuration src/move_robot_configuration.cpp)
add_library(move_robot_miscellaneous src/move_robot_miscellaneous.cpp)
add_library(move_robot src/move_robot.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_robot move_robot_miscellaneous move_robot_configuration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
#add_executable(move_robot_node src/move_robot_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(move_robot move_robot_gencfg)
add_dependencies(move_robot geometry_msgs_gencpp)


add_executable(move_robot_node
  src/move_robot_node.cpp
)
target_link_libraries(move_robot_node 
  ${catkin_LIBRARIES}
  move_robot
  
)
#add_dependencies(move_robot_node move_robot)
set_target_properties(move_robot_node PROPERTIES OUTPUT_NAME move_robot)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
  TARGETS 
    move_robot 
    move_robot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
