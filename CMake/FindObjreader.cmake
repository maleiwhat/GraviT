# Setting this to ON will ignore default paths and favor user selection.
# Set USE_NO_DEFAULT_PATH to OFF to avoid this functionality.
set(USE_NO_DEFAULT_PATH TRUE)
# For verbose output during the finding procedure.
set(VERBOSE_FIND FALSE)

set(OBJREADER_ENV_ROOT_DIR "$ENV{OBJREADER_ROOT_DIR}")

if(NOT OBJREADER_ROOT_DIR AND OBJREADER_ENV_ROOT_DIR)
  set(OBJREADER_ROOT_DIR "${OBJREADER_ENV_ROOT_DIR}")
endif(NOT OBJREADER_ROOT_DIR AND OBJREADER_ENV_ROOT_DIR)

message(STATUS "OBJREADER ROOT DIR  = ${OBJREADER_ROOT_DIR}")
if(OBJREADER_ROOT_DIR)
  set(OBJREADER_HEADER_SEARCH_DIRS "${OBJREADER_ROOT_DIR}"
    "${OBJREADER_ROOT_DIR}/include"
    ${OBJREADER_HEADER_SEARCH_DIRS})
endif(OBJREADER_ROOT_DIR)

find_path(OBJREADER_INCLUDE_DIR "objreader/objreader.h"
  PATHS ${OBJREADER_HEADER_SEARCH_DIRS})

# find libraries
set(OBJREADER_LIBRARY_SEARCH_DIRS
  "/opt/local/lib"
  "/usr/lib"
  "/usr/local/lib")

# add in what we know from OBJREADER_ROOT_DIR
if (VERBOSE_FIND)
  message(STATUS "OBJREADER_LIBRARY_SEARCH_DIRS = ${OBJREADER_LIBRARY_SEARCH_DIRS}")
endif()
if(OBJREADER_ROOT_DIR)
  set(OBJREADER_LIBRARY_SEARCH_DIRS  "${OBJREADER_ROOT_DIR}"
    "${OBJREADER_ROOT_DIR}/lib"
    ${OBJREADER_LIBRARY_SEARCH_DIRS})
endif(OBJREADER_ROOT_DIR)

if (VERBOSE_FIND)
  message(STATUS "OBJREADER_LIBRARY_SEARCH_DIRS = ${OBJREADER_LIBRARY_SEARCH_DIRS}")
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  find_library(OBJREADER_LIBRARY NAMES objreader
    PATHS "${OBJREADER_LIBRARY_SEARCH_DIRS}"
    PATH_SUFFIXES lib)
else()
  if (VERBOSE_FIND)
    message(STATUS "Using NO_DEFAULT_PATH")
  endif()
  if (USE_NO_DEFAULT_PATH)
    find_library(OBJREADER_LIBRARY NAMES objreader
      PATHS "${OBJREADER_ROOT_DIR}"
      PATH_SUFFIXES lib
      NO_DEFAULT_PATH)
  else()
    find_library(OBJREADER_LIBRARY NAMES objreader
      PATHS "${OBJREADER_ROOT_DIR}"
      PATH_SUFFIXES lib)
  endif()
endif()
message(STATUS "OBJREADER_INCLUDE_DIR = ${OBJREADER_INCLUDE_DIR}")
message(STATUS "OBJREADER_LIBRARY = ${OBJREADER_LIBRARY}")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Objreader REQUIRED_VARS
  OBJREADER_LIBRARY
  OBJREADER_INCLUDE_DIR)

message(STATUS "OBJREADER_FOUND = ${OBJREADER_FOUND}")

if(OBJREADER_FOUND)
  set(OBJREADER_FOUND "${OBJREADER_FOUND}")
  set(OBJREADER_INCLUDE_DIRS "${OBJREADER_INCLUDE_DIR}")
  set(OBJREADER_LIBRARIES "${OBJREADER_LIBRARY}")
endif(OBJREADER_FOUND)
