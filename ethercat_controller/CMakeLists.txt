cmake_minimum_required(VERSION 3.8)
project(ethercat_controller)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 禁用特定警告
  add_compile_options(-Wno-unused-parameter -Wno-missing-field-initializers -Wno-switch)
endif()

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# EtherCAT库路径
include_directories(/usr/local/etherlab/include)
link_directories(/usr/local/etherlab/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_library(MODBUS_LIBRARY modbus)
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(
  src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io_modules  # 新增IO模块路径
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fault_manager  # 新增故障管理模块路径
)

# 源文件
set(SOURCES
  src/main.cpp
  src/ethercat_node.cpp
  src/servo_axis_base.cpp
  src/leisai_servo_axis.cpp
  src/huichuan_servo_axis.cpp
  src/servo_axis_factory.cpp
  src/io_modules/io_interface.cpp  # IO模块
  src/io_modules/lights_controller.cpp  # 灯光控制器
  src/fault_manager/fault_management_system.cpp  # 故障管理系统
)

# 创建可执行文件
add_executable(ethercat_controller ${SOURCES})

# 链接依赖
ament_target_dependencies(ethercat_controller
    rclcpp
    std_msgs
    sensor_msgs
    ament_index_cpp
)

# 链接库
target_link_libraries(ethercat_controller
    /usr/local/etherlab/lib/libethercat.so
    ${MODBUS_LIBRARY}
    yaml-cpp
    pthread
    rt
)

# 安装可执行文件
install(TARGETS ethercat_controller
    DESTINATION lib/${PROJECT_NAME}
)

# 修复：安装包含文件（确保目录存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/include")
  install(DIRECTORY src/include/
    DESTINATION include/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/io_modules")
  install(DIRECTORY src/io_modules/
    DESTINATION include/${PROJECT_NAME}/io_modules
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/fault_manager")
  install(DIRECTORY src/fault_manager/
    DESTINATION include/${PROJECT_NAME}/fault_manager
  )
endif()

# 安装脚本文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/scripts/set_permissions.sh")
  install(PROGRAMS src/scripts/set_permissions.sh
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 安装 launch 文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/launch")
  install(DIRECTORY
    src/launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclcpp std_msgs sensor_msgs ament_index_cpp)
ament_export_include_directories(src/include)
ament_export_include_directories(src/io_modules)
ament_export_include_directories(src/fault_manager)

ament_package()
