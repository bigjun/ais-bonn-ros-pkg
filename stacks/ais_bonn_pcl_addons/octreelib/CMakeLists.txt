cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_check_for_sse ()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories (src)

set(SOURCES 
	src/octreelib/spatialaggregate/octree.cpp 
	src/octreelib/algorithm/downsample.cpp 
	src/octreelib/feature/normalestimation.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${SOURCES})
rosbuild_add_compile_flags (${PROJECT_NAME} ${SSE_FLAGS})

rosbuild_add_executable(test_octree test/test_octree.cpp)
rosbuild_add_compile_flags (test_octree ${SSE_FLAGS})

rosbuild_add_executable(test_normalestimation test/test_normalestimation.cpp)
rosbuild_add_compile_flags (test_normalestimation ${SSE_FLAGS})


#rosbuild_add_library( octreelib_normalestimation_nodelet
#		      src/octreelib/ros/multiresolution_feature.cpp
#                      src/octreelib/ros/normalestimation_nodelet.cpp
#                    )
#rosbuild_add_compile_flags( octreelib_normalestimation_nodelet ${SSE_FLAGS})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
