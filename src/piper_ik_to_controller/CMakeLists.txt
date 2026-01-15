cmake_minimum_required(VERSION 3.8)
project(piper_ik_to_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(piper_eval_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(osqp_vendor REQUIRED)

# --- OSQP Manual Linking ---
find_path(OSQP_INCLUDE_DIR NAMES osqp/osqp.h HINTS ${osqp_vendor_DIR}/../../../include NO_DEFAULT_PATH)
if(NOT OSQP_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find osqp/osqp.h")
endif()

find_library(OSQP_LIBRARY NAMES osqp HINTS ${osqp_vendor_DIR}/../../../lib NO_DEFAULT_PATH)
if(NOT OSQP_LIBRARY)
  message(FATAL_ERROR "Could not find libosqp.a/so")
endif()

# -------------------------------------------------------
# 1. Velocity IK Controller
# -------------------------------------------------------
add_executable(piper_ik_to_controller_node src/piper_ik_to_controller.cpp)
ament_target_dependencies(piper_ik_to_controller_node
  rclcpp geometry_msgs sensor_msgs trajectory_msgs kdl_parser orocos_kdl urdf piper_eval_msgs Eigen3
)
target_include_directories(piper_ik_to_controller_node PRIVATE ${OSQP_INCLUDE_DIR})
target_link_libraries(piper_ik_to_controller_node ${OSQP_LIBRARY})

# -------------------------------------------------------
# 2. Position IK Controller (The Fix is Here)
# -------------------------------------------------------
add_executable(piper_ik_to_controller_position src/piper_ik_to_controller_position.cpp)
ament_target_dependencies(piper_ik_to_controller_position
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  trajectory_msgs 
  kdl_parser 
  orocos_kdl 
  urdf 
  Eigen3
  piper_eval_msgs # <--- ADDED THIS MISSING DEPENDENCY
)
target_include_directories(piper_ik_to_controller_position PRIVATE ${OSQP_INCLUDE_DIR})
target_link_libraries(piper_ik_to_controller_position ${OSQP_LIBRARY})

# -------------------------------------------------------
# 3. Helper Nodes
# -------------------------------------------------------
add_executable(pose_filter_node src/pose_filter_node.cpp)
ament_target_dependencies(pose_filter_node rclcpp geometry_msgs)
target_include_directories(pose_filter_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(piper_ee_pose_publisher src/piper_ee_pose_publisher.cpp)
ament_target_dependencies(piper_ee_pose_publisher rclcpp geometry_msgs tf2 tf2_ros)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS 
  piper_ik_to_controller_node 
  piper_ik_to_controller_position 
  pose_filter_node
  piper_ee_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
