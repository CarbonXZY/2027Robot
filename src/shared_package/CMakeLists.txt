cmake_minimum_required(VERSION 3.8)
project(shared_package)

# alg_crc.h 需要 C++17（std::string_view / if constexpr / constexpr 静态成员）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 纯工具库，无 ROS 运行时依赖
find_package(ament_cmake REQUIRED)

# 把 Middleware（通信）编译成库，供其他包 find_package(shared_package) 后链接
add_library(shared_package SHARED Middleware/Communication_Interface.cpp)
target_compile_features(shared_package PUBLIC cxx_std_17)
target_include_directories(shared_package PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Middleware>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/device>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Algorithm>
  $<INSTALL_INTERFACE:include>)

install(TARGETS shared_package
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# 通信头（Middleware）+ 算法头（Algorithm）+ 设备结构体头（device）
install(FILES
  Middleware/Communication_Interface.hpp
  Algorithm/alg_crc.h
  device/motor_base.hpp
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
