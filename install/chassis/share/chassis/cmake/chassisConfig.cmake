# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_chassis_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED chassis_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(chassis_FOUND FALSE)
  elseif(NOT chassis_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(chassis_FOUND FALSE)
  endif()
  return()
endif()
set(_chassis_CONFIG_INCLUDED TRUE)

# output package information
if(NOT chassis_FIND_QUIETLY)
  message(STATUS "Found chassis: 0.0.0 (${chassis_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'chassis' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${chassis_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(chassis_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${chassis_DIR}/${_extra}")
endforeach()
