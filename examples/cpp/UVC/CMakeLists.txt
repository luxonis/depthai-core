project(uvc_examples)
cmake_minimum_required(VERSION 3.10)

add_subdirectory(uvc-gadget)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(uvc_example uvc_example.cpp OFF OFF)

target_link_libraries(uvc_example
    PRIVATE
    uvcgadget
)


# find_library(UVCGADGET_LIB
#   NAMES uvcgadget        # libuvc_gadget.so → -luvc_gadget
#   PATHS "${CMAKE_CURRENT_SOURCE_DIR}/uvc-gadget/build/lib"
#   NO_DEFAULT_PATH
# )

# # 2) Locate the headers (so we can #include "uvcgadget/stream.h")
# find_path(UVCGADGET_INC
#   NAMES uvcgadget/stream.h uvcgadget/depthai-source.h
#   PATHS "${CMAKE_CURRENT_SOURCE_DIR}/uvc-gadget/include"
#   NO_DEFAULT_PATH
# )

# # 3) Error out if we didn’t find them
# if(NOT UVCGADGET_LIB OR NOT UVCGADGET_INC)
#   message(FATAL_ERROR "Could not find uvc-gadget library or headers")
# endif()

# # 4) Tell the compiler where to look for headers…
# target_include_directories(uvc_example
#   PRIVATE
#     ${UVCGADGET_INC}
# )

# # 5) …and link the .so
# target_link_libraries(uvc_example
#   PRIVATE
#     ${UVCGADGET_LIB}
# )

# # 6) (Optional) make sure at runtime the loader can find your .so
# set_target_properties(uvc_example PROPERTIES
#   INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/uvc-gadget/build/lib"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )



# target_include_directories(uvc_example
#   PRIVATE
#     "${CMAKE_CURRENT_SOURCE_DIR}/uvc-gadget/include"
# )
