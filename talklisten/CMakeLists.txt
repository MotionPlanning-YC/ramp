# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(talklisten)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)
find_package(GTest REQUIRED)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS}) 

# Declare the executable, along with its source files.
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(utest test/utest.cpp)

# Specify libraries against which to link.
target_link_libraries(talker ${catkin_LIBRARIES} pthread)
target_link_libraries(listener ${catkin_LIBRARIES} pthread)
target_link_libraries(utest ${GTEST_LIBRARIES} pthread)

#================================ end
