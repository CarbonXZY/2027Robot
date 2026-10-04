cmake_minimum_required(VERSION 3.8)
# 目录按用户要求叫 Telecontrol；ROS2 包名必须小写，所以包名/project 名是 telecontrol。
project(telecontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(shared_package REQUIRED)
find_package(telecontrol_msgs REQUIRED)

# 遥控链路插件：hardware 传感器 + broadcaster 控制器，编译成一个库
add_library(telecontrol_hardware SHARED src/telecontrol_system.cpp)
target_compile_features(telecontrol_hardware PUBLIC cxx_std_17)
target_include_directories(telecontrol_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(telecontrol_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  shared_package
  telecontrol_msgs)

# 同一个 xml 给两类基类各注册一条
pluginlib_export_plugin_description_file(hardware_interface telecontrol_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface telecontrol_plugin.xml)

install(TARGETS telecontrol_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface controller_interface pluginlib shared_package telecontrol_msgs)

ament_package()
