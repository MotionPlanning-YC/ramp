cmake_minimum_required(VERSION 2.8.3)
project(ramp_planner)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation ramp_msgs roscpp)



#######################################
## Declare ROS messages and services ##
#######################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ramp_planner
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ~/catkin_workspace/src/ramp/trajectory_generator/src ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(ramp_planner src/main.cpp src/planner.cpp src/configuration.cpp src/path.cpp src/range.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(ramp_planner ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ramp_planner ramp_planner_generate_messages_cpp)

