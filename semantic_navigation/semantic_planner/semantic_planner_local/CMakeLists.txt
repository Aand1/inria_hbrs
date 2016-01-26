cmake_minimum_required(VERSION 2.8.3)
project(semantic_planner_local)

find_package(catkin REQUIRED 
        COMPONENTS
	    cmake_modules
            roscpp
            tf
            costmap_2d
	    base_local_planner
	    nav_core
	    message_generation
  
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES semantic_planner_local planner_local
    CATKIN_DEPENDS
        roscpp
        tf
        costmap_2d
        base_local_planner
	std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${trajectory_planner_ros_INCLUDE_DIRS}

)

add_library(semantic_planner_local 
  src/semantic_planner_local.cpp
)
target_link_libraries(semantic_planner_local
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}  
  ${Boost_LIBRARIES} 
)

add_library(planner_local 
  src/planner_local.cpp
)
target_link_libraries(planner_local
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}  
  ${Boost_LIBRARIES} 
  ${trajectory_planner_ros_LIBRARIES}
)


add_executable(semantic_planner_local_node
  src/semantic_planner_local_node.cpp
)
target_link_libraries(semantic_planner_local_node semantic_planner_local)
#set_target_properties(semantic_planner_local_node PROPERTIES OUTPUT_NAME semantic_planner_local)





