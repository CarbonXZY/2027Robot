#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "shared_package::shared_package" for configuration ""
set_property(TARGET shared_package::shared_package APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(shared_package::shared_package PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libshared_package.so"
  IMPORTED_SONAME_NOCONFIG "libshared_package.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS shared_package::shared_package )
list(APPEND _IMPORT_CHECK_FILES_FOR_shared_package::shared_package "${_IMPORT_PREFIX}/lib/libshared_package.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
