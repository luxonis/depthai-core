# Explicit list of public headers
set(uvcgadget_public_headers
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/configfs.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/events.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/list.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/stream.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/timer.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/v4l2-source.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/video-source.h
    ${CMAKE_SOURCE_DIR}/include/uvcgadget/depthai-source.h
)

# Install headers
install(FILES ${uvcgadget_public_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/uvcgadget)

# Expose variable to parent scope for library PUBLIC_HEADER
set(uvcgadget_public_headers ${uvcgadget_public_headers} PARENT_SCOPE)