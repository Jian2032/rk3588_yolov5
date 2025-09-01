cmake_minimum_required(VERSION 3.8)
project(robotic_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------
# 添加自定义头文件目录
# --------------------------
include_directories(
  include
  include/c_interface
  include/cpp_interface
  include/parameter
  include/robotic_arm_control
)
# --------------------------
# 可执行文件
# --------------------------
add_executable(robotic_arm_control_node
  src/robotic_arm_control.cpp
  src/control.cpp
)

# --------------------------
# 链接库（libnrc_host.so）
# --------------------------
target_link_libraries(robotic_arm_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libnrc_host.so
)

# --------------------------
# 添加 ROS2 依赖
# --------------------------
ament_target_dependencies(robotic_arm_control_node
  rclcpp
  std_msgs
)

# --------------------------
# 安装
# --------------------------
install(TARGETS
  robotic_arm_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库
install(FILES lib/libnrc_host.so DESTINATION lib)

ament_package()
