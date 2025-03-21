cmake_minimum_required(VERSION 3.8)
project(drone_commander_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg_pkg REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)


add_executable(drone_commander src/DroneCommander.cpp src/PosFunctions.cpp)
target_include_directories(drone_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(drone_commander
  rclcpp
  px4_msgs
  custom_msg_pkg
  geometry_msgs
)

install(
  TARGETS drone_commander
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs custom_msg_pkg)
ament_package()
