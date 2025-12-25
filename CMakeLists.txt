cmake_minimum_required(VERSION 3.8)
project(ros2_drone_core)

# 【关键】自动生成 compile_commands.json，让 CLion 能看懂代码
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# 1. 编译封装库 (Wrapper Library)
add_library(px4_wrapper_lib src/px4_wrapper.cpp)
target_include_directories(px4_wrapper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_wrapper_lib rclcpp px4_msgs)

# 2. 编译主节点 (Executable)
add_executable(motion_controller src/motion_node.cpp)
target_link_libraries(motion_controller px4_wrapper_lib)
ament_target_dependencies(motion_controller rclcpp px4_msgs)

# 安装规则
install(TARGETS px4_wrapper_lib motion_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
