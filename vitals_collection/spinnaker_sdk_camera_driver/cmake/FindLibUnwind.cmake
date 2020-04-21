# Find the libunwind library
#
#  LibUnwind_FOUND        - True if libunwind was found.
#  LibUnwind_LIBRARIES    - The libraries needed to use libunwind
#  LibUnwind_INCLUDE_DIRS - Location of unwind.h and libunwind.h

FIND_PATH(LibUnwind_INCLUDE_DIRS libunwind.h)
if(NOT LibUnwind_INCLUDE_DIRS)
  message(STATUS "failed to find libunwind.h")
elseif(NOT EXISTS "${LibUnwind_INCLUDE_DIRS}/unwind.h")
  message(STATUS "libunwind.h was found, but unwind.h was not found in that directory.")
  SET(LibUnwind_INCLUDE_DIRS "")
endif()

FIND_LIBRARY(LibUnwind_GENERIC_LIBRARY "unwind")
if(NOT LibUnwind_GENERIC_LIBRARY)
    MESSAGE(STATUS "failed to find unwind generic library")
endif()
SET(LibUnwind_LIBRARIES ${LibUnwind_GENERIC_LIBRARY})

# For some reason, we have to link to two libunwind shared object files:
# one arch-specific and one not.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    SET(LibUnwind_ARCH "arm")
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    SET(LibUnwind_ARCH "x86_64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^i.86$")
    SET(LibUnwind_ARCH "x86")
endif()

if(LibUnwind_ARCH)
    FIND_LIBRARY(LibUnwind_SPECIFIC_LIBRARY "unwind-${LibUnwind_ARCH}")
    if (NOT LibUnwind_SPECIFIC_LIBRARY)
        MESSAGE(STATUS "failed to find unwind-${LibUnwind_ARCH}")
    endif()
    SET(LibUnwind_LIBRARIES ${LibUnwind_LIBRARIES} ${LibUnwind_SPECIFIC_LIBRARY})
endif(LibUnwind_ARCH)

MARK_AS_ADVANCED(LibUnwind_LIBRARIES LibUnwind_INCLUDE_DIRS)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(LibUnwind DEFAULT_MSG
  LibUnwind_LIBRARIES LibUnwind_INCLUDE_DIRS)
