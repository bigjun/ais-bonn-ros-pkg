cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(octreelib)


# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# check for SSE flags
include(${PROJECT_SOURCE_DIR}/cmake/octreelib_find_sse.cmake)
OCTREELIB_CHECK_FOR_SSE()

add_definitions( ${SSE_FLAGS} )


# Boost (required)
include(${PROJECT_SOURCE_DIR}/cmake/octreelib_find_boost.cmake)


# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)


# PCL (required)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



include_directories( ${PROJECT_SOURCE_DIR}/include )

set(SOURCES
        src/octreelib/spatialaggregate/octree.cpp
        src/octreelib/algorithm/downsample.cpp
        src/octreelib/feature/normalestimation.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})


