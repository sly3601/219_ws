#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hardware_unitree_mujoco::hardware_unitree_mujoco" for configuration ""
set_property(TARGET hardware_unitree_mujoco::hardware_unitree_mujoco APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hardware_unitree_mujoco::hardware_unitree_mujoco PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhardware_unitree_mujoco.so"
  IMPORTED_SONAME_NOCONFIG "libhardware_unitree_mujoco.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_unitree_mujoco::hardware_unitree_mujoco )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_unitree_mujoco::hardware_unitree_mujoco "${_IMPORT_PREFIX}/lib/libhardware_unitree_mujoco.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
