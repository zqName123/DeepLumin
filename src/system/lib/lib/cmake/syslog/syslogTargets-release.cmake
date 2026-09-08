#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "syslog::syslog" for configuration "Release"
set_property(TARGET syslog::syslog APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(syslog::syslog PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsyslog.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS syslog::syslog )
list(APPEND _IMPORT_CHECK_FILES_FOR_syslog::syslog "${_IMPORT_PREFIX}/lib/libsyslog.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
