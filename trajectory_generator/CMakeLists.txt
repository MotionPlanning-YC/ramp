cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation roscpp trajectory_msgs)


#######################################
## Declare ROS messages and services ##
#######################################


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS geometry_msgs message_generation roscpp trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(trajectory_generator src/main.cpp src/trajectory.cpp src/motion_state.cpp src/segment.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(trajectory_generator trajectory_generator_generate_messages_cpp)

