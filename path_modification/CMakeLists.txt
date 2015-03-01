cmake_minimum_required(VERSION 2.8.3)
project(path_modification)

find_package(catkin REQUIRED COMPONENTS message_generation ramp_msgs roscpp)


#######################################
## Declare ROS messages and services ##
#######################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/change.cpp src/constraints.cpp src/crossover.cpp src/delete.cpp src/insert.cpp src/modifier.cpp src/swap.cpp src/utility.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)
