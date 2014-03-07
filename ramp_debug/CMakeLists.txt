cmake_minimum_required(VERSION 2.8.3)
project(ramp_debug)

find_package(catkin REQUIRED roscpp nav_msgs ramp_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}) #/usr/share/include/eigen3)

add_executable(display_base_location src/display_base_location.cpp src/configuration.cpp src/range.cpp src/utility.cpp)
set_target_properties(display_base_location PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(display_base_location ${catkin_LIBRARIES})
add_dependencies(display_base_location ramp_msgs_generate_messages_cpp)


add_executable(viewPredicted src/viewPredicted.cpp src/configuration.cpp src/range.cpp src/utility.cpp src/trajectory_request_handler.cpp)
set_target_properties(viewPredicted PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(viewPredicted ${catkin_LIBRARIES})
add_dependencies(viewPredicted ramp_msgs_generate_messages_cpp)


add_executable(publishTwistCommand src/publish_twist_command.cpp)
set_target_properties(publishTwistCommand PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(publishTwistCommand ${catkin_LIBRARIES})
add_dependencies(publishTwistCommand ramp_msgs_generate_messages_cpp)


add_executable(publishTrajectoryCommand src/publish_trajectory_command.cpp)
set_target_properties(publishTrajectoryCommand PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(publishTrajectoryCommand ${catkin_LIBRARIES})
add_dependencies(publishTrajectoryCommand ramp_msgs_generate_messages_cpp)

add_executable(getYaw src/getYaw.cpp)
set_target_properties(getYaw PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(getYaw ${catkin_LIBRARIES})
add_dependencies(getYaw ramp_msgs_generate_messages_cpp)
