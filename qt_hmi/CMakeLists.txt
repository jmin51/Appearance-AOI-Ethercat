cmake_minimum_required(VERSION 3.8)
project(qt_hmi)

# 默认C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 源文件列表
set(SOURCES
  src/main.cpp
  src/ros2_bridge.cpp
  src/main_window.cpp
  src/jog_bar.cpp
  src/io_panel.cpp
  src/axis_panel.cpp
  src/manual_mode_page.cpp
  src/auto_mode_page.cpp
)

# 启用AUTOMOC (必须在add_executable之前设置，Qt元对象编译器自动处理Q_OBJECT类)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 构建可执行目标
add_executable(qt_hmi_node ${SOURCES})

# 链接依赖
ament_target_dependencies(qt_hmi_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)
target_link_libraries(qt_hmi_node Qt5::Widgets yaml-cpp)

# 安装目标
install(TARGETS qt_hmi_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
