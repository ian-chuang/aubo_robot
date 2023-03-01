cmake_minimum_required(VERSION 2.8.3)
project(aubo_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  aubo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs aubo_msgs
)


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include include/otg ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${catkin_LIB_DIRS}  lib/lib64 lib/lib64/config lib/lib64/aubocontroller)
else()
        link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()




add_executable(teach_controller
    src/teach_controller.cpp
    src/aubo_driver.cpp
)
add_dependencies(teach_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teach_controller pthread ${catkin_LIBRARIES} auborobotcontroller otgLib)















