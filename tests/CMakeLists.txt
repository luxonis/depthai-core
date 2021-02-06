# Add CMake testing infrastructure
include(CTest)
enable_testing()

# Enable sanitizers when testing is enabled
set(SANITIZE_ADDRESS ON CACHE BOOL "Enable AddressSanitizer for sanitized targets." FORCE)
set(SANITIZE_UNDEFINED ON CACHE BOOL "Enable UndefinedBehaviorSanitizer for sanitized targets." FORCE)

# Enable sanitizer for library
find_package(Sanitizers)
add_sanitizers(${TARGET_NAME})

# Add catch2 for writing tests
hunter_add_package(Catch)
find_package(Catch2 CONFIG REQUIRED)

# Macro for adding new tests
macro(dai_add_test test_name test_src)
    add_executable("${test_name}" "${test_src}")
    set_property(TARGET ${test_name} PROPERTY CXX_STANDARD 14)
    target_link_libraries("${test_name}" PRIVATE depthai-core Catch2::Catch2)
    add_test("${test_name}" "${test_name}")
    # Add sanitizers for tests as well
    add_sanitizers(${test_name})
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

dai_add_test(neural_network_test src/neural_network_test.cpp)
target_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")



