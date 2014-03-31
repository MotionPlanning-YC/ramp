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

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

##We first select every .cpp files in the src/TypeIIRML folder
file(GLOB_RECURSE TypeIIRML_SRC "src/TypeIIRML/*.cpp")

## Declare a cpp library
add_library(TypeIIRML ${TypeIIRML_SRC})

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/Reflexxes.cpp src/utility.cpp src/range.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} TypeIIRML)


