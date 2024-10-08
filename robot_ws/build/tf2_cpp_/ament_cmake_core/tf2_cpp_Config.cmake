# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tf2_cpp__CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tf2_cpp__FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tf2_cpp__FOUND FALSE)
  elseif(NOT tf2_cpp__FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tf2_cpp__FOUND FALSE)
  endif()
  return()
endif()
set(_tf2_cpp__CONFIG_INCLUDED TRUE)

# output package information
if(NOT tf2_cpp__FIND_QUIETLY)
  message(STATUS "Found tf2_cpp_: 0.0.0 (${tf2_cpp__DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tf2_cpp_' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tf2_cpp__DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tf2_cpp__FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tf2_cpp__DIR}/${_extra}")
endforeach()
