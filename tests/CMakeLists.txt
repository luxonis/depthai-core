# Add CMake testing infrastructure
include(CTest)
enable_testing()

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
    # Add ubsan halt on error
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1")
endmacro() 

# Mobilenet network
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
    SHA1 "f0e14978b3f77a4f93b9f969cd39e58bb7aef490"
    FILE "mobilenet-ssd_openvino_2021.2_6shave.blob"
    LOCATION mobilenet_blob
)

# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)

dai_add_test(neural_network_test src/neural_network_test.cpp)
target_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")



