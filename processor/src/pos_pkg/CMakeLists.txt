cmake_minimum_required(VERSION 3.8)
project(pos_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(position_monitor src/PosFunctions.cpp)
ament_target_dependencies(position_monitor
  rclcpp
  px4_msgs
)

install(
  TARGETS position_monitor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs)
ament_package()
