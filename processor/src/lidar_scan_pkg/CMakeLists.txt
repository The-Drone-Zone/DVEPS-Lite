cmake_minimum_required(VERSION 3.8)
project(lidar_scan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(RPLIDAR_SDK_PATH "${PROJECT_SOURCE_DIR}/sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

add_executable(lidar_node	src/lidar_pub_node.cpp ${RPLIDAR_SDK_SRC})
# target_link_libraries(rplidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(lidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(lidar_composition src/lidar_pub_node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(lidar_composition
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(lidar_analysis_node src/lidar_analysis_node.cpp)
ament_target_dependencies(lidar_analysis_node
  rclcpp
  std_srvs
  sensor_msgs
)

install(
  TARGETS lidar_node lidar_composition lidar_analysis_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
