cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. 海康 SDK 头文件和库路径
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

# 3. 添加可执行文件
add_executable(hik_camera_node src/hik_camera_node.cpp)

# 4. 链接 ROS 2 依赖
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
)

# 5. 链接海康的库
target_link_libraries(hik_camera_node
  MvCameraControl
)

# 6. 安装规则
install(TARGETS
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 7. 测试相关（保留原装配置）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})