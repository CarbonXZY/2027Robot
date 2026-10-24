cmake_minimum_required(VERSION 3.8)
project(robot)

find_package(ament_cmake REQUIRED)

# 纯 bringup 包：只装 URDF 和 launch（无 C++ 代码）
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
