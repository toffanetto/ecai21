# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_potential_field_control_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED potential_field_control_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(potential_field_control_FOUND FALSE)
  elseif(NOT potential_field_control_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(potential_field_control_FOUND FALSE)
  endif()
  return()
endif()
set(_potential_field_control_CONFIG_INCLUDED TRUE)

# output package information
if(NOT potential_field_control_FIND_QUIETLY)
  message(STATUS "Found potential_field_control: 0.0.0 (${potential_field_control_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'potential_field_control' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${potential_field_control_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(potential_field_control_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${potential_field_control_DIR}/${_extra}")
endforeach()
