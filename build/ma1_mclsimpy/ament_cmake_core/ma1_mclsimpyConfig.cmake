# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ma1_mclsimpy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ma1_mclsimpy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ma1_mclsimpy_FOUND FALSE)
  elseif(NOT ma1_mclsimpy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ma1_mclsimpy_FOUND FALSE)
  endif()
  return()
endif()
set(_ma1_mclsimpy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ma1_mclsimpy_FIND_QUIETLY)
  message(STATUS "Found ma1_mclsimpy: 0.0.0 (${ma1_mclsimpy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ma1_mclsimpy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ma1_mclsimpy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ma1_mclsimpy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ma1_mclsimpy_DIR}/${_extra}")
endforeach()
