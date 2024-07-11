cmake_minimum_required(VERSION 3.0.2)
project(connectwo_bringup)

# add_compile_options(-std=c++11)

# find_package(catkin REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  serial
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(motor_controller_rpi src/motor_controller_rpi.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(motor_controller_rpi ${catkin_LIBRARIES})
