#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "depthai::core" for configuration "Debug"
set_property(TARGET depthai::core APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(depthai::core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C;CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdepthai-core.a"
  )

list(APPEND _cmake_import_check_targets depthai::core )
list(APPEND _cmake_import_check_files_for_depthai::core "${_IMPORT_PREFIX}/lib/libdepthai-core.a" )

# Import target "depthai::opencv" for configuration "Debug"
set_property(TARGET depthai::opencv APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(depthai::opencv PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdepthai-opencv.a"
  )

list(APPEND _cmake_import_check_targets depthai::opencv )
list(APPEND _cmake_import_check_files_for_depthai::opencv "${_IMPORT_PREFIX}/lib/libdepthai-opencv.a" )

# Import target "depthai::depthai-resources" for configuration "Debug"
set_property(TARGET depthai::depthai-resources APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(depthai::depthai-resources PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdepthai-resources.a"
  )

list(APPEND _cmake_import_check_targets depthai::depthai-resources )
list(APPEND _cmake_import_check_files_for_depthai::depthai-resources "${_IMPORT_PREFIX}/lib/libdepthai-resources.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
