#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "reference_filter_dp::reference_filter_dp_component" for configuration ""
set_property(TARGET reference_filter_dp::reference_filter_dp_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(reference_filter_dp::reference_filter_dp_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libreference_filter_dp_component.so"
  IMPORTED_SONAME_NOCONFIG "libreference_filter_dp_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS reference_filter_dp::reference_filter_dp_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_reference_filter_dp::reference_filter_dp_component "${_IMPORT_PREFIX}/lib/libreference_filter_dp_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
