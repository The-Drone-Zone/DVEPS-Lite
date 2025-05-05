cmake_minimum_required(VERSION 3.8)
project(decision_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CUDA REQUIRED)
find_package(camera_scan_pkg)
find_package(custom_msg_pkg REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  camera_scan_pkg
  custom_msg_pkg
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS
  decision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()
