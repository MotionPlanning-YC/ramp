cmake_minimum_required(VERSION 2.8.3)
project(ramp_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation nav_msgs std_msgs trajectory_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES KnotPoint.msg MotionState.msg Path.msg Range.msg RampTrajectory.msg Obstacle.msg Population.msg)

## Generate services in the 'srv' folder
add_service_files(FILES EvaluationRequest.srv ModificationRequest.srv TrajectoryRequest.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES nav_msgs std_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS message_runtime nav_msgs std_msgs trajectory_msgs)

