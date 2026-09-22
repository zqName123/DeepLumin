cmake_minimum_required(VERSION 3.10)
if(NOT SYSLOG_SKIP_PROJECT)
  project(syslog VERSION 1.0.0 LANGUAGES CXX)
endif()

find_package(Threads REQUIRED)

add_library(syslog STATIC
  src/logger.cpp
)

target_include_directories(syslog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(syslog PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(syslog PUBLIC Threads::Threads)

add_library(syslog::syslog ALIAS syslog)

install(TARGETS syslog
  EXPORT syslogTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(EXPORT syslogTargets
  FILE syslogTargets.cmake
  NAMESPACE syslog::
  DESTINATION lib/cmake/syslog
)

# include(CMakePackageConfigHelpers)

# configure_package_config_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/syslogConfig.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/syslogConfig.cmake
#   INSTALL_DESTINATION lib/cmake/syslog
# )

# write_basic_package_version_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/syslogConfigVersion.cmake
#   VERSION 1.0.0
#   COMPATIBILITY SameMajorVersion
# )

# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/syslogConfig.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}/syslogConfigVersion.cmake
#   DESTINATION lib/cmake/syslog
# )
