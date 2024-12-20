cmake_minimum_required(VERSION 3.20)

include(compileShader.cmake)

# create shader directory in build
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/shaders")
set(SHADER_HEADER_PATH "${CMAKE_BINARY_DIR}/include/depthai/shaders")
# vulkan_compile_shader generates headers in source directory
compile_shader(
  INFILE rgbd2pointcloud.comp OUTFILE
  ${SHADER_HEADER_PATH}/rgbd2pointcloud.hpp NAMESPACE
  "dai::shaders")
add_library(shaders INTERFACE
            ${SHADER_HEADER_PATH}/rgbd2pointcloud.hpp)

target_include_directories(
  shaders INTERFACE $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
                    $<INSTALL_INTERFACE:include/shaders>)

