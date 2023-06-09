# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_open_cam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED open_cam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(open_cam_FOUND FALSE)
  elseif(NOT open_cam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(open_cam_FOUND FALSE)
  endif()
  return()
endif()
set(_open_cam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT open_cam_FIND_QUIETLY)
  message(STATUS "Found open_cam: 0.0.0 (${open_cam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'open_cam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${open_cam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(open_cam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${open_cam_DIR}/${_extra}")
endforeach()
