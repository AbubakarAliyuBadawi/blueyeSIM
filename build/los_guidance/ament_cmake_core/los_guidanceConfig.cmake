# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_los_guidance_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED los_guidance_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(los_guidance_FOUND FALSE)
  elseif(NOT los_guidance_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(los_guidance_FOUND FALSE)
  endif()
  return()
endif()
set(_los_guidance_CONFIG_INCLUDED TRUE)

# output package information
if(NOT los_guidance_FIND_QUIETLY)
  message(STATUS "Found los_guidance: 0.0.0 (${los_guidance_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'los_guidance' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${los_guidance_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(los_guidance_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${los_guidance_DIR}/${_extra}")
endforeach()
