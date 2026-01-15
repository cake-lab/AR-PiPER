cmake_minimum_required(VERSION 3.8)
project(piper_eval_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for all messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PiperTeleopMetric.msg"
  "msg/PiperRmseMetrics.msg"  # <-- NEW
  "srv/PiperResetMetrics.srv" # <-- NEW
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

