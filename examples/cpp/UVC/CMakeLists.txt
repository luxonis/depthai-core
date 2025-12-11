project(uvc_examples)
cmake_minimum_required(VERSION 3.10)

add_subdirectory(uvc-gadget)

dai_add_example(uvc_example uvc_example.cpp OFF OFF)

target_link_libraries(uvc_example
    PRIVATE
    uvcgadget
)
