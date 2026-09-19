#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "chassis::chassis_hardware" for configuration ""
set_property(TARGET chassis::chassis_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(chassis::chassis_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libchassis_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libchassis_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS chassis::chassis_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_chassis::chassis_hardware "${_IMPORT_PREFIX}/lib/libchassis_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
