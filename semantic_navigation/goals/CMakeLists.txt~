cmake_minimum_required(VERSION 2.8.3)
project(goals)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sem_nav_msgs
  cmake_modules
  actionlib
)

catkin_package(
 CATKIN_DEPENDS 
   roscpp        
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${roscpp_INCLUDE_DIRS}
  ${sem_nav_msgs_INCLUDE_DIRS} 
)

add_executable(use_case_one
 src/use_case_one.cpp
)
target_link_libraries(use_case_one
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_two
 src/use_case_two.cpp
)
target_link_libraries(use_case_two
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_four
 src/use_case_four.cpp
)
target_link_libraries(use_case_four
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_three_one
 src/use_case_three_one.cpp
)
target_link_libraries(use_case_three_one
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_six
 src/use_case_six.cpp
)
target_link_libraries(use_case_six
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_seven
 src/use_case_seven.cpp
)
target_link_libraries(use_case_seven
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)

add_executable(use_case_eight
 src/use_case_eight.cpp
)
target_link_libraries(use_case_eight
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${sem_nav_msgs_LIBRARIES} 
)





