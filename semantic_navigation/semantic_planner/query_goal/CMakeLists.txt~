# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8.3)

# Declare the name of the CMake Project
project(query_goal)

find_package(catkin REQUIRED COMPONENTS
  semantic_knowledgebase
  roscpp
)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES query_goal
  CATKIN_DEPENDS 
    semantic_knowledgebase 
    roscpp
)

include_directories(include)
include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(query_goal
   src/client_goal.cpp
)
# Define a library target called client_library
#add_executable(client_library src/client_library.cpp)
#target_link_libraries(client_library ${roscpp_LIBRARIES})
target_link_libraries(query_goal
   ${catkin_LIBRARIES}
)


# Define a node target called client_library_node
add_executable(client_goal_node src/client_library_node.cpp)
target_link_libraries(client_goal_node ${catkin_LIBRARIES} query_goal)
