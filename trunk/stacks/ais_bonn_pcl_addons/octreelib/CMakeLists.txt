cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(octreelib)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories (src)

set(SOURCES
        src/octreelib/spatialaggregate/octree.cpp
        src/octreelib/algorithm/downsample.cpp
        src/octreelib/feature/normalestimation.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${SOURCES})
rosbuild_add_compile_flags (${PROJECT_NAME} ${SSE_FLAGS})


