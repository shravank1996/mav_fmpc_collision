cmake_minimum_required(VERSION 2.8.3)
project(mav_low_level_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  asl_msgs
  mav_msgs
  roscpp
  sensor_msgs
  catkin_simple
)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

cs_add_library(low_level_controller_lib
  src/low_level_control.cpp)

cs_add_executable(low_level_control_node
  src/low_level_control_node.cpp
)

target_link_libraries(low_level_control_node low_level_controller_lib)

cs_add_library(pid_low_level_controller_lib
  src/pid_low_level_control.cpp)

cs_add_executable(pid_low_level_control_node
  src/pid_low_level_control_node.cpp
)

target_link_libraries(pid_low_level_control_node pid_low_level_controller_lib)

catkin_package()

cs_install()
cs_export()

