cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H723VGT6)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings (must come after project())
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    User/Module/Gripper/mod_gripper.cpp
    User/Module/Gripper/mod_gripper.h
    User/Device/test/motor_dji.cpp
    User/Device/test/motor_dji.h
    User/Algorithm/alg_mit.cpp
    User/Algorithm/alg_mit.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# CMSIS DSP library for STM32H7 (Cortex-M7, little-endian, single-precision FPU)
set(CMSIS_DSP_LIB_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/ARM)
set(CMSIS_DSP_LIB     ${CMSIS_DSP_LIB_DIR}/arm_cortexM7lfsp_math.lib)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMSIS_DSP_LIB_DIR}
)

# Add sources to executable
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User/Algorithm/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Algorithm/*.c
    ${CMAKE_SOURCE_DIR}/User/Module/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Module/*.c
    ${CMAKE_SOURCE_DIR}/User/Device/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Device/*.c
    ${CMAKE_SOURCE_DIR}/User/Driver/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Driver/*.c
    ${CMAKE_SOURCE_DIR}/User/Interaction/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Interaction/*.c
    ${CMAKE_SOURCE_DIR}/User/Middleware/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Middleware/*.c
    ${CMAKE_SOURCE_DIR}/User/Task/*.cpp
    ${CMAKE_SOURCE_DIR}/User/Task/*.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/User/Algorithm
    ${CMAKE_SOURCE_DIR}/User/Module
    ${CMAKE_SOURCE_DIR}/User/Module/Leg
    ${CMAKE_SOURCE_DIR}/User/Device
    ${CMAKE_SOURCE_DIR}/User/Driver
    ${CMAKE_SOURCE_DIR}/User/Interaction
    ${CMAKE_SOURCE_DIR}/User/Middleware
    ${CMAKE_SOURCE_DIR}/User/Task
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # CMSIS DSP library for STM32H7
    ${CMSIS_DSP_LIB}
)
