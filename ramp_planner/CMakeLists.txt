cmake_minimum_required(VERSION 2.8.3)
project(ramp_planner)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation roscpp trajectory_msgs)



#######################################
## Declare ROS messages and services ##
#######################################


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ramp_planner
  CATKIN_DEPENDS geometry_msgs message_generation roscpp trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(ramp_planner src/main.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(ramp_planner ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ramp_planner ramp_planner_generate_messages_cpp)

