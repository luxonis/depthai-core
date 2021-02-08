# Add catch2 for writing tests
hunter_add_package(Catch)
find_package(Catch2 CONFIG REQUIRED)

option(DEPTHAI_SANITIZER "Build tests with Clang sanitizer" OFF)

if(DEPTHAI_SANITIZER)

    set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address,undefined -fno-omit-frame-pointer" CACHE STRING
    "Flags used by the C compiler for Asan build type or configuration." FORCE)

    set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined -fno-omit-frame-pointer" CACHE STRING
    "Flags used by the C++ compiler for Asan build type or configuration." FORCE)

    set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} -fsanitize=address,undefined" CACHE STRING
    "Linker flags to be used to create executables for Asan build type." FORCE)

    set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} -fsanitize=address,undefined" CACHE STRING
    "Linker lags to be used to create shared libraries for Asan build type." FORCE)

endif()


# Macro for adding new tests
macro(dai_add_test test_name test_src)
    add_executable("${test_name}" "${test_src}")
    set_property(TARGET ${test_name} PROPERTY CXX_STANDARD 14)
    target_link_libraries("${test_name}" PRIVATE depthai-core Catch2::Catch2)
    add_test("${test_name}" "${test_name}")
endmacro() 

# Mobilenet network
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet.blob"
    SHA1 "e89d3ee9f26d80397e44f89c6b375990064a4a42"
    FILE "mobilenet.blob"
    LOCATION mobilenet_blob
)

# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)

dai_add_test(neural_network src/neural_network_test.cpp)
target_compile_definitions(neural_network PRIVATE BLOB_PATH="${mobilenet_blob}")



