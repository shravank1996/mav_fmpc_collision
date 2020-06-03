cmake_minimum_required(VERSION 3.1)
project(mav_fmpc_multi_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  asl_msgs
  catkin_simple
  mav_msgs
  roscpp
  tf
)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)



cs_add_library(mpc_queue_lib src/mpc_queue.cpp)
cs_add_library(ekf_drag_estimator_lib src/ekf_drag_estimator.cpp)
cs_add_library(fmpc_lib src/fmpc.cpp
  solver/qpoases/src/BLASReplacement.cpp
  solver/qpoases/src/Bounds.cpp
  solver/qpoases/src/Constraints.cpp
  solver/qpoases/src/Flipper.cpp
  solver/qpoases/src/Indexlist.cpp
  solver/qpoases/src/LAPACKReplacement.cpp
  solver/qpoases/src/Matrices.cpp
  solver/qpoases/src/MessageHandling.cpp
  solver/qpoases/src/Options.cpp
  solver/qpoases/src/QProblem.cpp
  solver/qpoases/src/QProblemB.cpp
  solver/qpoases/src/SubjectTo.cpp
  solver/qpoases/src/Utils.cpp
  solver/qpoases/src/SolutionAnalysis.cpp
  solver/qpoases/src/OQPinterface.cpp
  solver/qpoases/src/SQProblem.cpp
  solver/qpoases/src/SparseSolver.cpp
  solver/qpoases/src/SQProblemSchur.cpp
  solver/qpoases/src/SubjectTo.cpp
  )

target_include_directories(fmpc_lib PUBLIC
        solver
	solver/qpoases
	solver/qpoases/include
	solver/qpoases/src
	)


cs_add_executable(fmpc_node
  src/fmpc_node.cpp
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

target_link_libraries(fmpc_node ${catkin_LIBRARIES} fmpc_lib ekf_drag_estimator_lib mpc_queue_lib)
target_link_libraries(fmpc_node osqp::osqp)
target_link_libraries(fmpc_node OsqpEigen::OsqpEigen)

cs_add_executable(thrust_publish src/thrust_pub.cpp)
target_link_libraries(thrust_publish ${catkin_LIBRARIES})

cs_add_executable(collision_detection src/robot_collision_detection.cpp)
target_link_libraries(collision_detection ${catkin_LIBRARIES})

cs_add_executable(waypoint_publisher src/trajectory_publisher/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})

catkin_package()

cs_install()
cs_export()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


