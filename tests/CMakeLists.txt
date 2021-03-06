# Add CMake testing infrastructure
include(CTest)
enable_testing()

# Add catch2 for writing tests
hunter_add_package(Catch)
find_package(Catch2 CONFIG REQUIRED)

# Macro for adding new tests
function(dai_add_test test_name test_src)

    # Create test executable
    add_executable(${test_name} ${test_src})

    # Add to clangformat target
    target_clangformat_setup(${test_name} "")

    # Link to core and Catch2 testing framework
    target_link_libraries(${test_name} PRIVATE depthai-core Catch2::Catch2)

    # Add to list of tests
    add_test(${test_name} ${test_name})

    # Add sanitizers for tests as well
    add_sanitizers(${test_name})

    # Add ubsan halt on error
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1")
endfunction()

# Mobilenet network
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_8shave.blob"
    SHA1 "3329bb8f3a9c881ef9756d232055f9d6f38aa07b"
    FILE "mobilenet-ssd_openvino_2021.2_8shave.blob"
    LOCATION mobilenet_blob
)

# OpenVINO 2020.3 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.3_4shave.blob"
    SHA1 "f0134c9b843fe414f6d98b17a70f069d1ab0f3d8"
    FILE "text-image-super-resolution-0001_2020.3_4shave.blob"
    LOCATION openvino_2020_3_blob
)
# OpenVINO 2020.4 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.4_4shave.blob"
    SHA1 "25dcf0b146da8c85c9c4cba00ad5fdd4ed02a1b6"
    FILE "text-image-super-resolution-0001_2020.4_4shave.blob"
    LOCATION openvino_2020_4_blob
)

# OpenVINO 2021.1 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.1_4shave.blob"
    SHA1 "39c4f47f2a75627b7561e97dd7cdfcd0b1925a1e"
    FILE "text-image-super-resolution-0001_2021.1_4shave.blob"
    LOCATION openvino_2021_1_blob
)
# OpenVINO 2021.2 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.2_4shave.blob"
    SHA1 "a204467f86aa4ad63d31782ada271bea6f57f789"
    FILE "text-image-super-resolution-0001_2021.2_4shave.blob"
    LOCATION openvino_2021_2_blob
)
# OpenVINO 2021.3 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.3_4shave.blob"
    SHA1 "af19470feb59317e74d045bc31d93ca129c46674"
    FILE "text-image-super-resolution-0001_2021.3_4shave.blob"
    LOCATION openvino_2021_3_blob
)

# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)

dai_add_test(neural_network_test src/neural_network_test.cpp)
target_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_test(openvino_blob src/openvino_blob_test.cpp)
target_compile_definitions(openvino_blob PRIVATE
    OPENVINO_2020_3_BLOB_PATH="${openvino_2020_3_blob}"
    OPENVINO_2020_4_BLOB_PATH="${openvino_2020_4_blob}"
    OPENVINO_2021_1_BLOB_PATH="${openvino_2021_1_blob}"
    OPENVINO_2021_2_BLOB_PATH="${openvino_2021_2_blob}"
    OPENVINO_2021_3_BLOB_PATH="${openvino_2021_3_blob}"
)



