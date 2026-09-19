cmake_minimum_required(VERSION 3.8)
project(chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(shared_package REQUIRED)

# ros2_control 硬件接口插件（编译成动态库，由 pluginlib 加载）
add_library(chassis_hardware SHARED src/chassis_system.cpp)
target_compile_features(chassis_hardware PUBLIC cxx_std_17)
target_include_directories(chassis_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(chassis_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  shared_package)

# 注册为 hardware_interface 插件
pluginlib_export_plugin_description_file(hardware_interface chassis_plugin.xml)

install(TARGETS chassis_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib shared_package)

ament_package()
