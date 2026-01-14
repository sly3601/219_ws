# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_keyboard_input_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED keyboard_input_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(keyboard_input_FOUND FALSE)
  elseif(NOT keyboard_input_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(keyboard_input_FOUND FALSE)
  endif()
  return()
endif()
set(_keyboard_input_CONFIG_INCLUDED TRUE)

# output package information
if(NOT keyboard_input_FIND_QUIETLY)
  message(STATUS "Found keyboard_input: 0.0.0 (${keyboard_input_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'keyboard_input' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${keyboard_input_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(keyboard_input_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${keyboard_input_DIR}/${_extra}")
endforeach()
