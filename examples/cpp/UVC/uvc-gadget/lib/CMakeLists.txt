# SPDX-License-Identifier: CC0-1.0

#cmakedefine HAVE_LIBCAMERA
#cmakedefine CONFIG_CAN_ENCODE


# File: lib/CMakeLists.txt
add_library(uvcgadget SHARED
    configfs.c
    events.c
    jpg-source.c
    slideshow-source.c
    stream.c
    test-source.c
    timer.c
    uvc.c
    v4l2.c
    v4l2-source.c
    video-buffers.c
    video-source.c
    depthai-source.c
)

# if(libcamera_FOUND)
#     target_sources(uvcgadget PRIVATE libcamera-source.cpp)
#     target_link_libraries(uvcgadget PRIVATE ${libcamera_LIBRARIES})
# endif()

# if(JPEG_FOUND AND Threads_FOUND)
#     target_sources(uvcgadget PRIVATE mjpeg_encoder.cpp)
#     target_link_libraries(uvcgadget PRIVATE JPEG::JPEG Threads::Threads)
# endif()

target_include_directories(uvcgadget
  PUBLIC
    # so that video-source.c / v4l2-source.c etc can see their .h files:
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    # your public API:
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/uvcgadget>
    # generated headers (config.h) in the build tree:
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include>
    # installation path
    $<INSTALL_INTERFACE:include>
)

set_target_properties(uvcgadget PROPERTIES
    VERSION ${uvc_gadget_version}
    SOVERSION ${PROJECT_VERSION}
)

install(TARGETS uvcgadget
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Optional pkg-config generation (requires a .pc.in template):
# configure_file(${CMAKE_SOURCE_DIR}/lib/uvcgadget.pc.in ${CMAKE_BINARY_DIR}/uvcgadget.pc @ONLY)
# install(FILES ${CMAKE_BINARY_DIR}/uvcgadget.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
