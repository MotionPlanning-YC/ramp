cmake_minimum_required(VERSION 2.8.3)
project(ramp_planner)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation ramp_msgs roscpp)



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

# Specify include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/planner.cpp src/configuration.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)

