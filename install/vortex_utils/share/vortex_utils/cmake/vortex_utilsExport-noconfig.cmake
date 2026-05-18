#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vortex_utils::vortex_utils" for configuration ""
set_property(TARGET vortex_utils::vortex_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(vortex_utils::vortex_utils PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libvortex_utils.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS vortex_utils::vortex_utils )
list(APPEND _IMPORT_CHECK_FILES_FOR_vortex_utils::vortex_utils "${_IMPORT_PREFIX}/lib/libvortex_utils.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
