cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_collision_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  asl_msgs
  catkin_simple
  mav_msgs
  roscpp
)


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(multi_robot_collision_detection_lib
  src/multi_robot_collision_detection.cpp)

cs_add_executable(multi_robot_collision_detection_node
  src/multi_robot_collision_detection_node.cpp
)

target_link_libraries(multi_robot_collision_detection_node multi_robot_collision_detection_lib)


catkin_package()

cs_install()
cs_export()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)





