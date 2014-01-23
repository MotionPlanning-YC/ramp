cmake_minimum_required(VERSION 2.8.3)
project(evaluation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation ramp_msgs roscpp corobot_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES ramp_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/collision_detection.cpp src/euclidean_distance.cpp src/evaluate.cpp src/time.cpp src/trajectory_request_handler.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ramp_msgs_generate_message_cpp corobot_msgs_generate_message_cpp)
