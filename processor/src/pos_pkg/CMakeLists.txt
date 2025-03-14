cmake_minimum_required(VERSION 3.8)
project(pos_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(position_monitor src/PosFunctions.cpp)
target_include_directories(position_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(position_monitor
  rclcpp
  px4_msgs
  geometry_msgs
)

install(
  TARGETS position_monitor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pos_functions_lib)
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs)
ament_package()
