# Add CMake testing infrastructure
include(CTest)
enable_testing()

# Add catch2 for writing tests
hunter_add_package(Catch2)
find_package(Catch2 CONFIG REQUIRED)

# Function for adding new tests
# Creates separate tests for USB and PoE devices
function(dai_add_test test_name test_src)

    # Create test executable
    add_executable(${test_name} ${test_src})
    add_default_flags(${test_name} LEAN)

    # Add to clangformat target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${test_name} "")
    endif()

    # Link to core and Catch2 testing framework
    target_link_libraries(${test_name} PRIVATE depthai-core Catch2::Catch2)

    # Add sanitizers for tests as well
    if(COMMAND add_sanitizers)
        add_sanitizers(${test_name})
    endif()

    # Add to list of tests
    add_test(${test_name} ${test_name})
    # Add ubsan halt on error
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=usb" LABELS usb)

    # Check if MSVC and add a conforming preprocessor test (serialization macros)
    if (MSVC)
        # Create test executable with MSVC conforming preprocessor
        add_executable(${test_name}_conforming ${test_src})

        # Link to core and Catch2 testing framework
        target_link_libraries(${test_name}_conforming PRIVATE depthai-core Catch2::Catch2)

        if(MSVC_VERSION LESS 1925)
            target_compile_options(${test_name}_conforming INTERFACE "/experimental:preprocessor")
        else()
            target_compile_options(${test_name}_conforming INTERFACE "/Zc:preprocessor")
        endif()

        # Add to list of tests
        add_test(${test_name}_conforming ${test_name}_conforming)

        # Add ubsan halt on error
        set_tests_properties(${test_name}_conforming PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=usb" LABELS usb)
    endif()

    # Add PoE test variants
    add_test(${test_name}_poe ${test_name})
    # Add ubsan halt on error
    set_tests_properties(${test_name}_poe PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=tcpip;DEPTHAI_SEARCH_TIME=15000" LABELS poe)
    if(MSVC)
        # Add to list of tests
        add_test(${test_name}_conforming_poe ${test_name}_conforming)
        # Add ubsan halt on error
        set_tests_properties(${test_name}_conforming_poe PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=tcpip;DEPTHAI_SEARCH_TIME=15000" LABELS poe)
    endif()

endfunction()

# Function for adding compile definitions to tests
function(dai_test_compile_definitions)
    target_compile_definitions(${ARGV})
    message(STATUS "Calling first with ${ARGV}")
    if(MSVC)
        set(ARGV0 ${ARGV0}_conforming)
        list(REMOVE_AT ARGV 0)
        list(INSERT ARGV 0 ${ARGV0})
        message(STATUS "Calling second with ${ARGV}")
        target_compile_definitions(${ARGV})
    endif()
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
# OpenVINO 2021.4 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.4_4shave.blob"
    SHA1 "fd11bb63781fc3315cdf4bb74247a8b98817f6fd"
    FILE "text-image-super-resolution-0001_2021.4_4shave.blob"
    LOCATION openvino_2021_4_blob
)


# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)

dai_add_test(neural_network_test src/neural_network_test.cpp)
dai_test_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_test(openvino_blob_test src/openvino_blob_test.cpp)
dai_test_compile_definitions(openvino_blob_test PRIVATE
    OPENVINO_2020_3_BLOB_PATH="${openvino_2020_3_blob}"
    OPENVINO_2020_4_BLOB_PATH="${openvino_2020_4_blob}"
    OPENVINO_2021_1_BLOB_PATH="${openvino_2021_1_blob}"
    OPENVINO_2021_2_BLOB_PATH="${openvino_2021_2_blob}"
    OPENVINO_2021_3_BLOB_PATH="${openvino_2021_3_blob}"
    OPENVINO_2021_4_BLOB_PATH="${openvino_2021_4_blob}"
)

# Bootloader configuration tests
dai_add_test(bootloader_config_test src/bootloader_config_test.cpp)

# Device USB Speed test
dai_add_test(device_usbspeed_test src/device_usbspeed_test.cpp)

# Unlimited io connections test
dai_add_test(unlimited_io_connection_test src/unlimited_io_connection_test.cpp)

# Serialization test
dai_add_test(serialization_test src/serialization_test.cpp)

# Multiple devices test
dai_add_test(multiple_devices_test src/multiple_devices_test.cpp)
