# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ublox_dgnss_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ublox_dgnss_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ublox_dgnss_FOUND FALSE)
  elseif(NOT ublox_dgnss_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ublox_dgnss_FOUND FALSE)
  endif()
  return()
endif()
set(_ublox_dgnss_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ublox_dgnss_FIND_QUIETLY)
  message(STATUS "Found ublox_dgnss: 0.5.2 (${ublox_dgnss_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ublox_dgnss' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ublox_dgnss_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ublox_dgnss_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ublox_dgnss_DIR}/${_extra}")
endforeach()
