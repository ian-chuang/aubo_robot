cmake_minimum_required(VERSION 3.0.2)
project(aubo_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  joint_limits_interface
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp controller_manager hardware_interface joint_limits_interface sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aubo_hardware_interface src/aubo_hardware_interface.cpp src/aubo_hardware_interface_obj.cpp)
target_link_libraries(aubo_hardware_interface ${catkin_LIBRARIES})
add_dependencies(aubo_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS aubo_hardware_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)