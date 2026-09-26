cmake_minimum_required(VERSION 3.8)
project(control_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
