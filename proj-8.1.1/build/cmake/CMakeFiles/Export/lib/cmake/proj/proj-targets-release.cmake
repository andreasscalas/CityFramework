#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PROJ::proj" for configuration "Release"
set_property(TARGET PROJ::proj APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PROJ::proj PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libproj.so.22.1.1"
  IMPORTED_SONAME_RELEASE "libproj.so.22"
  )

list(APPEND _IMPORT_CHECK_TARGETS PROJ::proj )
list(APPEND _IMPORT_CHECK_FILES_FOR_PROJ::proj "${_IMPORT_PREFIX}/lib/libproj.so.22.1.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
