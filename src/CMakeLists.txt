cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

project(tdc VERSION 0.0.2)

find_package(OpenCV REQUIRED)
find_package(jetson-utils)
find_package(CUDA REQUIRED)
message("-- CUDA version: ${CUDA_VERSION}")
find_package(JetsonGPIO REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4 gstreamer-sdp-1.0>=1.4 gstreamer-video-1.0>=1.4 gstreamer-app-1.0>=1.4)
message("-- GSTREAMER version: ${GST_VERSION}")

message("-- GST_APP_LIBRARIES:${GST_APP_LIBRARIES}")
message("-- GST_VIDEO_LIBRARIES:${GST_VIDEO_LIBRARIES}")


#######################################################
######################## Build ########################
#######################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${JetsonGPIO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} main.cpp image_converter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${JetsonGPIO_LIBRARIES} jetson-utils jetson-inference ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES})








## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
