cmake_minimum_required(VERSION 3.8)
project(comm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shared_package REQUIRED)

# 独立节点（带 main），专门用来单测「中间件 + 驱动」这一层
add_executable(comm_test_node src/comm_test_node.cpp)
target_compile_features(comm_test_node PRIVATE cxx_std_17)
ament_target_dependencies(comm_test_node
  rclcpp
  shared_package)

install(TARGETS comm_test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
