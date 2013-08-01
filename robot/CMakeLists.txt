cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation nav_msgs roscpp trajectory_msgs)

#######################################
## Declare ROS messages and services ##
#######################################


generate_messages(DEPENDENCIES trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot src/main.cpp src/corobot.cpp src/base_configuration.cpp src/utility.cpp)
target_link_libraries(robot ${catkin_LIBRARIES})
add_dependencies(robot robot_generate_messages_cpp)
