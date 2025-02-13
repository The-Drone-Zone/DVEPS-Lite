cmake_minimum_required(VERSION 3.8)
project(laser_scan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(LIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk")
include_directories(${LIDAR_SDK_DIR}/include)
link_directories(${LIDAR_SDK_DIR}/src)
message(STATUS "LIDAR SDK Directory: ${LIDAR_SDK_DIR}")

set(LIB_SDK_A "${CMAKE_CURRENT_SOURCE_DIR}/../../output/Linux/Release")
message(STATUS "LIDAR SDK Directory: ${LIB_SDK_A}")

# Add the RPLidar publisher node
add_executable(lidar_pub_node src/lidar_pub_node.cpp)
ament_target_dependencies(lidar_pub_node rclcpp sensor_msgs std_srvs)
target_link_libraries(lidar_pub_node ${LIB_SDK_A}/libsl_lidar_sdk.a)

# Add the Lidar subscriber node
add_executable(lidar_sub_node src/lidar_sub_node.cpp)
ament_target_dependencies(lidar_sub_node rclcpp sensor_msgs)

install(TARGETS lidar_pub_node lidar_sub_node DESTINATION lib/${PROJECT_NAME})

ament_package()
