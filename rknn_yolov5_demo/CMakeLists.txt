cmake_minimum_required(VERSION 3.8)
project(rknn_yolov5_demo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread")

# =======================
# ROS2 依赖
# =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(robot_msgs REQUIRED)

# =======================
# RKNN/RGA 路径
# =======================
set(LIB_ARCH aarch64)

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/../../runtime/RK3588/${CMAKE_SYSTEM_NAME}/librknn_api)
set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/include/librknnrt.so)

# rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/include/3rdparty/rga/RK3588)
set(RGA_LIB ${RGA_PATH}/lib/Linux/${LIB_ARCH}/librga.so)

# =======================
# 包含头文件
# =======================
include_directories(
  ${RKNN_API_PATH}/include
  ${CMAKE_SOURCE_DIR}/include/3rdparty
  ${RGA_PATH}/include
  ${CMAKE_SOURCE_DIR}/include
  ${realsense2_INCLUDE_DIR}
)

# =======================
# 源文件
# =======================
add_executable(rknn_yolov5_node
  src/main.cc
  src/rknnyolo.cc
  src/rkYolov5s.cc
  src/preprocess.cc
  src/postprocess.cc
)

# =======================
# 链接库
# =======================
target_link_libraries(rknn_yolov5_node
  ${RKNN_RT_LIB}
  ${OpenCV_LIBS}
  ${RGA_LIB}
)

ament_target_dependencies(rknn_yolov5_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  robot_msgs
)

# =======================
# 安装
# =======================
install(TARGETS rknn_yolov5_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
install(PROGRAMS ${RGA_LIB} DESTINATION lib)
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
