cmake_minimum_required(VERSION 3.8)
project(camera_scan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_APPSINK_ONLY "Use only appsink in GStreamer pipeline (no display)" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CUDA REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0>=1.14
  gstreamer-base-1.0
  gstreamer-app-1.0
)


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

rosidl_generate_interfaces(camera_scan_pkg
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(img_pub_node src/img_raw_pub_node.cpp)

if(USE_APPSINK_ONLY)
  target_compile_definitions(img_pub_node PRIVATE USE_APPSINK_ONLY)
endif()

ament_target_dependencies(img_pub_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_include_directories(img_pub_node PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(img_pub_node ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES})


add_executable(img_analysis_node src/img_analysis_node.cpp)
ament_target_dependencies(img_analysis_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  geometry_msgs
  rosidl_default_runtime
)

rosidl_target_interfaces(img_analysis_node
  camera_scan_pkg "rosidl_typesupport_cpp")

# target_link_libraries(img_pub_node ${OpenCV_LIBS})


include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # For generated message headers
include_directories(
  ${GSTREAMER_INCLUDE_DIRS}
  include
)

link_directories(${OpenCV_LIB_DIR} ${CUDA_LIBRARIES})
target_link_libraries(img_analysis_node ${OpenCV_LIBS} opencv_core opencv_imgproc opencv_cudaimgproc)

install(TARGETS
  img_pub_node
  img_analysis_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()