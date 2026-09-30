cmake_minimum_required(VERSION 3.8)
project(robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
