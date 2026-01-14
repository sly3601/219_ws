#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "unitree_guide_controller::unitree_guide_controller" for configuration "Release"
set_property(TARGET unitree_guide_controller::unitree_guide_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(unitree_guide_controller::unitree_guide_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/unitree_guide_controller/libunitree_guide_controller.so"
  IMPORTED_SONAME_RELEASE "libunitree_guide_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS unitree_guide_controller::unitree_guide_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_unitree_guide_controller::unitree_guide_controller "${_IMPORT_PREFIX}/lib/unitree_guide_controller/libunitree_guide_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
