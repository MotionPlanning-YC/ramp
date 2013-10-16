cmake_minimum_required(VERSION 2.8.3)
project(sensing)

find_package(catkin REQUIRED COMPONENTS corobot_msgs message_generation ramp_msgs roscpp tf)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES corobot_msgs ramp_msgs)

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
add_executable(${PROJECT_NAME} src/main.cpp src/dynamic_object.cpp src/ir_object.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

