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
find_package(custom_msg_pkg REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../mavlink
)

add_executable(lidar_pub_node	src/lidar_pub_node.cpp ${RPLIDAR_SDK_SRC})
# target_link_libraries(rplidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(lidar_pub_node
  rclcpp
  std_srvs
  sensor_msgs
  custom_msg_pkg
)

# add_executable(lidar_composition src/lidar_pub_node.cpp ${RPLIDAR_SDK_SRC})
# ament_target_dependencies(lidar_composition
#   rclcpp
#   std_srvs
#   sensor_msgs
#   custom_msg_pkg
# )

add_executable(lidar_analysis_node src/lidar_analysis_node.cpp)
ament_target_dependencies(lidar_analysis_node
  rclcpp
  std_srvs
  sensor_msgs
  custom_msg_pkg
  px4_msgs
)

add_executable(obstacle_distance_node src/obstacle_distance_node.cpp)
ament_target_dependencies(obstacle_distance_node 
  rclcpp
  sensor_msgs
)

install(
  TARGETS lidar_pub_node  lidar_analysis_node obstacle_distance_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
) #lidar_composition: PROBABLY DELETE WE ARE NOT USING THIS NODE

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(px4_msgs)
ament_export_include_directories(include)
ament_package()
