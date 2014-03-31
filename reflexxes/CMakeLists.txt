cmake_minimum_required(VERSION 2.8.3)
project(reflexxes)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reflexxes
  CATKIN_DEPENDS roscpp nav_msgs tf
  ## DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/Reflexxes)
add_executable(${PROJECT_NAME} src/main.cpp src/reflexxes.cpp src/utility.cpp src/range.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} TypeIIRML ReflexxesTypeIV)

