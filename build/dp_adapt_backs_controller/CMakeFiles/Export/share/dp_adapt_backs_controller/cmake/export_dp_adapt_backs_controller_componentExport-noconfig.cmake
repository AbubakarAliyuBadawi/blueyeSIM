#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dp_adapt_backs_controller::dp_adapt_backs_controller_component" for configuration ""
set_property(TARGET dp_adapt_backs_controller::dp_adapt_backs_controller_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dp_adapt_backs_controller::dp_adapt_backs_controller_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdp_adapt_backs_controller_component.so"
  IMPORTED_SONAME_NOCONFIG "libdp_adapt_backs_controller_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dp_adapt_backs_controller::dp_adapt_backs_controller_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_dp_adapt_backs_controller::dp_adapt_backs_controller_component "${_IMPORT_PREFIX}/lib/libdp_adapt_backs_controller_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
