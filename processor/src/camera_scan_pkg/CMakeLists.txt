cmake_minimum_required(VERSION 3.8)
project(camera_scan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(CUDA REQUIRED)

file(READ "/proc/1/cgroup" cgroup_content)
if(cgroup_content MATCHES "docker")
    set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
else()
    set(OpenCV_DIR "/ssd/opencv/lib/cmake/opencv4")
endif()

# Find OpenCV 4.10
find_package(OpenCV 4.10 REQUIRED)

# # Specify the OpenCV version you want to use
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Opencv Include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "CUDA Include dirs: ${CUDA_INCLUDE_DIRS}")


add_executable(img_pub_node src/img_raw_pub_node.cpp)
ament_target_dependencies(img_pub_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

add_executable(img_sub_node src/img_raw_sub_node.cpp)
ament_target_dependencies(img_sub_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

link_directories(${OpenCV_LIB_DIR} ${CUDA_LIBRARIES})
target_link_libraries(img_sub_node ${OpenCV_LIBS} opencv_core opencv_imgproc opencv_cudaimgproc)

install(TARGETS
  img_pub_node
  img_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
ament_export_include_directories(include)
ament_package()
