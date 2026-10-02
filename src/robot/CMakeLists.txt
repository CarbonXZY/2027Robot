cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(telecontrol_msgs REQUIRED)

# 人机交互口子：遥控帧 → 底盘 Twist
add_executable(robot src/robot.cpp)
target_compile_features(robot PUBLIC cxx_std_17)
target_include_directories(robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot
  rclcpp
  geometry_msgs
  telecontrol_msgs)

install(TARGETS robot DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
