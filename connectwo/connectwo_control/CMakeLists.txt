cmake_minimum_required(VERSION 3.0.2)
project(connectwo_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  hardware_interface
  joint_state_controller
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/connectwo_control
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES connectwo_control
  CATKIN_DEPENDS controller_manager effort_controllers hardware_interface joint_state_controller roscpp std_msgs sensor_msgs nav_msgs tf tf2 
  DEPENDS Boost
)

add_executable(motor_controller_pc src/motor_controller_pc.cpp)
target_link_libraries(motor_controller_pc ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} pthread)

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} pthread)

add_executable(cmd_vel_amplifier src/cmd_vel_amplifier.cpp)
target_link_libraries(cmd_vel_amplifier ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} pthread)

add_executable(connectwo_control src/connectwo_control.cpp)
target_link_libraries(connectwo_control ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} pthread
)


## Install targets
install(TARGETS motor_controller_pc odom_publisher cmd_vel_amplifier connectwo_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
