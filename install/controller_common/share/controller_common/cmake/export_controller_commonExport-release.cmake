#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "controller_common::controller_common" for configuration "Release"
set_property(TARGET controller_common::controller_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(controller_common::controller_common PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcontroller_common.so"
  IMPORTED_SONAME_RELEASE "libcontroller_common.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS controller_common::controller_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_controller_common::controller_common "${_IMPORT_PREFIX}/lib/libcontroller_common.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
