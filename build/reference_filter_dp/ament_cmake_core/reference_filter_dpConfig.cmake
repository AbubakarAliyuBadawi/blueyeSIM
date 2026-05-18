# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_reference_filter_dp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED reference_filter_dp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(reference_filter_dp_FOUND FALSE)
  elseif(NOT reference_filter_dp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(reference_filter_dp_FOUND FALSE)
  endif()
  return()
endif()
set(_reference_filter_dp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT reference_filter_dp_FIND_QUIETLY)
  message(STATUS "Found reference_filter_dp: 0.0.0 (${reference_filter_dp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'reference_filter_dp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${reference_filter_dp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(reference_filter_dp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${reference_filter_dp_DIR}/${_extra}")
endforeach()
