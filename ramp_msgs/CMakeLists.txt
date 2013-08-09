cmake_minimum_required(VERSION 2.8.3)
project(ramp_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg geometry_msgs std_msgs trajectory_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES Configuration.msg ModificationRequest.msg Path.msg Trajectory.msg TrajectoryRequest.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs trajectory_msgs)

