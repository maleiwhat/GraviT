##################################################
#
#	Root CMakeLists file for project GraviT
#

PROJECT(GraviT)
#MESSAGE("Project source dir ${PROJECT_SOURCE_DIR}")
#MESSAGE("Project binary dir ${PROJECT_BINARY_DIR}")
#MESSAGE("Project GraviT source dir ${GraviT_SOURCE_DIR}")
#MESSAGE("Project GraviT binary dir ${GraviT_BINARY_DIR}")
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
IF(COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

#FIND_LIBRARY(MPE_LIBRARIES NAME libmpe.a PATHS "/home/01197/semeraro/mpe2-1.3.0/build/lib" NO_DEFAULT_PATH)
# PROJECT_BASE_DIR not used
# modified to use GraviT SOURCE_DIR instead of PROJECT_SOURCE_DIR to 
# avoid ambiguity. 
#SET(PROJECT_BASE_DIR ${PROJECT_SOURCE_DIR})
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_BASE_DIR}/CMake)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${GraviT_SOURCE_DIR}/CMake)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type: Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

#GVT_XXX_DIR are never used outside of this file. 
# comment them out and modify the statements in this
# file that rely on GVT_XXX_DIR
#SET(GVT_BINARY_DIR ${PROJECT_BINARY_DIR})
#SET(GVT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
# modify following statement to use GraviT_BINARY_DIR
# instead of GVT_BINARY_DIR
#SET(LIBRARY_OUTPUT_PATH ${GVT_BINARY_DIR}/lib)
#SET(EXECUTABLE_OUTPUT_PATH ${GVT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${GraviT_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${GraviT_BINARY_DIR}/bin)
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)

SET(GVT_ICC OFF CACHE BOOL "Use Intel Compiler?")
MARK_AS_ADVANCED(GVT_ICC)
SET(GVT_MPI ON CACHE BOOL "Use MPI")
SET(GVT_MPE OFF CACHE BOOL "Turn on MPE Profiling")
SET(GVT_XEON_TARGET "AVX" CACHE STRING "Target ISA on host (SSE,AVX,AVX2)")

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  SET(GVT_COMPILER ${CMAKE_CXX_COMPILER_ID} CACHE STRING "GraviT Compiler?")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  SET(GVT_COMPILER ${CMAKE_CXX_COMPILER_ID} CACHE STRING "GraviT Compiler?")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  SET(GVT_COMPILER ${CMAKE_CXX_COMPILER_ID} CACHE STRING "GraviT Compiler?")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  SET(GVT_COMPILER ${CMAKE_CXX_COMPILER_ID} CACHE STRING "GraviT Compiler?")
ENDIF()


##############################################################
# modules
##############################################################
# gravit.cmake is not necessary. The macro therein is never used
#INCLUDE(CMake/gravit.cmake)
INCLUDE(CMake/mpi.cmake)
INCLUDE(CMake/boost.cmake)
INCLUDE(CMake/doxygen.cmake)

#FIND_PACKAGE(OptiX REQUIRED)
#INCLUDE_DIRECTORIES(${OptiX_INCLUDE})
#SET(LIBS ${LIBS} ${optix_LIBRARY})

#This command changes the value of the PROJECT_SOURCE_DIR
#Keep the variable unchanged and modify the following 
#commands to work properly. IE in what follows this commented
# line use ${PROJECT_SOURCE_DIR}/src where ${PROJECT_SOURCE_DIR} is expected.
#SET(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# This line was changed to conform to not changing the PROJECT_SOURCE_DIR
# variable. And added GraviT_BINARY_DIR to include directories. This
# allows build to find config/config.h
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${GraviT_SOURCE_DIR}/src ${GraviT_BINARY_DIR})

SET (GVT_TARGET ${GVT_XEON_TARGET})
#These commands were also changed to reflect the new policy toward PROJECT_SOURCE_DIR
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/GVT builddir/gravit/)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/Backend builddir/Backend/)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/Frontend builddir/Frontend/)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/apps builddir/apps/)

ADD_SUBDIRECTORY(${GraviT_SOURCE_DIR}/src/GVT builddir/gravit/)
ADD_SUBDIRECTORY(${GraviT_SOURCE_DIR}/src/Backend builddir/Backend/)
ADD_SUBDIRECTORY(${GraviT_SOURCE_DIR}/src/Frontend builddir/Frontend/)
ADD_SUBDIRECTORY(${GraviT_SOURCE_DIR}/src/apps builddir/apps/)
#Modified due to not needing to define GVT_SOURCE_DIR
#CONFIGURE_FILE("${GVT_SOURCE_DIR}/config/config.h.in" "${GVT_BINARY_DIR}/config/config.h")
CONFIGURE_FILE("${GraviT_SOURCE_DIR}/src/config/config.h.in" "${GraviT_BINARY_DIR}/config/config.h")


FIND_PACKAGE(Objreader REQUIRED)
