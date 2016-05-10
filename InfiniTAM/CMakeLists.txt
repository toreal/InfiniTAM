cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED )

IF(MSVC_IDE)
  set(FADE_INCLUDE_DIR  C:/James/fadeRelease/include_fade2d)
  set(FADE_LIB_DIR C:/James/fadeRelease/x64 )
  set(FADE_LIBS fade2D_x64_v140_Debug.lib )
  include_directories(${FADE_INCLUDE_DIR} )
ENDIF(MSVC_IDE)

#find_package(OpenMesh)
#set(OPENMESH_INCLUDE_DIR  C:/James/OpenMesh51/include)
#set(OPENMESH_LIB_DIR C:/James/OpenMesh51/lib )
#set(OPENMESH_INCLUDE_DIR  /usr/local/include)
#set(OPENMESH_LIB_DIR /usr/local/lib )
#set(OPENMESH_LIBS /usr/local/lib/libOpenMeshCore.a /usr/local/lib/libOpenMeshTools.dylib)
#include_directories(${OPENMESH_INCLUDE_DIR} )


find_package(GLEW)
CANONIFY_BOOL(GLEW_FOUND)
message(STATUS "GLEW found: " ${GLEW_FOUND})

find_package(CGAL COMPONENTS Core)
CANONIFY_BOOL(CGAL_FOUND)
message(STATUS "CGAL found: " ${CGAL_FOUND})

include( ${CGAL_USE_FILE} )

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS )
  add_definitions(-DUSING_CMAKE=1)

  LINK_DIRECTORIES(C:/james/openglLIB/lib ${FADE_LIB_DIR}  $ENV{RSSDK_DIR}/lib/x64 )
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
#add_definitions(-DINCLUDE_TEMPLATES)

  LINK_DIRECTORIES( ${OPENMESH_LIB_DIR} )

ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)


#IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
#ENDIF()

add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(Utils)
add_subdirectory(Engine)


IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)
target_link_libraries(InfiniTAM_cli ${OpenCV_LIBS} ${CGAL_LIBS})
add_executable(InfiniTAM InfiniTAM.cpp )

IF(MSVC_IDE)
  target_link_libraries(InfiniTAM_cli ${FADE_LIBS} glew32.lib )
  target_link_libraries(InfiniTAM ${FADE_LIBS} libpxc_d.lib glew32.lib )
ENDIF(MSVC_IDE)
target_link_libraries(InfiniTAM ${OpenCV_LIBS} ${CGAL_LIBRARIES} ${GLEW_LIBRARIES} )
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)

