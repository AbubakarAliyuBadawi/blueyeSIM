#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "thrust_allocator_auv::thrust_allocator_auv_component" for configuration ""
set_property(TARGET thrust_allocator_auv::thrust_allocator_auv_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(thrust_allocator_auv::thrust_allocator_auv_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthrust_allocator_auv_component.so"
  IMPORTED_SONAME_NOCONFIG "libthrust_allocator_auv_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS thrust_allocator_auv::thrust_allocator_auv_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_thrust_allocator_auv::thrust_allocator_auv_component "${_IMPORT_PREFIX}/lib/libthrust_allocator_auv_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
