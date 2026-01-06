# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Sanitizers)
find_package(argparse REQUIRED)

# Create utility library
add_library(utility utility/utility.cpp)
target_compile_features(utility PUBLIC cxx_std_17)
target_include_directories(utility PUBLIC "utility" "$<BUILD_INTERFACE:${FP16_INCLUDE_DIR}>")
add_default_flags(utility LEAN)
target_link_libraries(utility ${OpenCV_LIBS})

set(test_env
    # Misc
    "UBSAN_OPTIONS=halt_on_error=1"
    "RUNNING_AS_TEST=1"
    # DepthAI
    "DEPTHAI_SEARCH_TIMEOUT=20000"
    "DEPTHAI_CONNECT_TIMEOUT=10000"
    "DEPTHAI_RECONNECT_TIMEOUT=0"
    )

set(TOP_CPP_EXAMPLES_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Regexes for example test failure
set(STRICT_EXAMPLE_TEST_FAIL_REGEX "\\[warning\\];\\[error\\];\\[critical\\]")
set(RELAXED_EXAMPLE_TEST_FAIL_REGEX "\\[error\\];\\[critical\\]")

# Helper for adding new examples
function(dai_add_example example_name example_src enable_test use_pcl)

    # Add example
    add_executable(${example_name} ${example_src})
    add_default_flags(${example_name} LEAN)
    set(DEPTHAI_TARGET depthai::core)
    if(use_pcl AND NOT DEPTHAI_MERGED_TARGET)
        set(DEPTHAI_TARGET depthai::all)
    elseif(NOT DEPTHAI_MERGED_TARGET)
        set(DEPTHAI_TARGET depthai::opencv)
    endif()
    target_link_libraries(${example_name} PRIVATE utility ${DEPTHAI_TARGET} ${OpenCV_LIBS} Threads::Threads argparse::argparse)
    # Set compiler features (c++17), and disables extensions (g++17)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD 17)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD_REQUIRED ON)
    set_property(TARGET ${example_name} PROPERTY CXX_EXTENSIONS OFF)

    # Add sanitizers for example
    if(COMMAND add_sanitizers)
        add_sanitizers(${example_name})
    endif()
    # Add to clangformat target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${example_name} "")
    endif()

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1 2 3)

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(${enable_test} AND DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND
            ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=300
            -P ${TOP_CPP_EXAMPLES_CMAKE_DIR}/cmake/ExecuteTestTimeout.cmake
            $<TARGET_FILE:test_wrapper>
            40 # Set test timeout
            $<TARGET_FILE:${example_name}>
            ${arguments}
        )

        add_dependencies(${example_name} test_wrapper)
        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "${STRICT_EXAMPLE_TEST_FAIL_REGEX}")

        # Add ubsan halt on error
        set_tests_properties(${example_name} PROPERTIES ENVIRONMENT "${test_env}")

    endif()

    # Copy over required DLLs (Windows)
    if(WIN32)
        # Copy dlls to target directory - Windows only
        # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
        if(CMAKE_VERSION VERSION_LESS "3.21")
            file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        else()
            set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${example_name}>")
        endif()
        add_custom_command(TARGET ${example_name} POST_BUILD COMMAND
            "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${example_name}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
    endif()
endfunction()

function(dai_example_test_relax_fail_regex example_name)
    if(DEPTHAI_TEST_EXAMPLES AND TEST ${example_name})
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "${RELAXED_EXAMPLE_TEST_FAIL_REGEX}")
    endif()
endfunction()

# Function for setting test labels, including label shortcuts
function(dai_set_example_test_labels example_name)
    # If the example is not enabled for testing, return
    if(NOT DEPTHAI_TEST_EXAMPLES)
        return()
    endif()
    set(expanded_labels "")
    foreach(label IN LISTS ARGN)
        if(label STREQUAL "rvc2_all")
            list(APPEND expanded_labels "rvc2" "poe" "usb")
        else()
            list(APPEND expanded_labels ${label})
        endif()
    endforeach()
    # Add "cpp_example" label
    list(APPEND expanded_labels "cpp_example")
    set_tests_properties(${example_name} PROPERTIES LABELS "${expanded_labels}")
endfunction()

function(dai_install_file example_name file_name)
    file(COPY ${CMAKE_CURRENT_LIST_DIR}/${file_name} DESTINATION ".")  # copy file to where the example is built
endfunction()

add_subdirectory(AprilTags)
add_subdirectory(Benchmark)
add_subdirectory(Camera)
add_subdirectory(DetectionNetwork)

if(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT)
    add_subdirectory(DynamicCalibration)
endif()

add_subdirectory(Events)
add_subdirectory(FeatureTracker)
add_subdirectory(ObjectTracker)
add_subdirectory(HostNodes)
add_subdirectory(ImageManip)
add_subdirectory(IMU)
add_subdirectory(Misc)
add_subdirectory(ModelZoo)
add_subdirectory(NeuralNetwork)
add_subdirectory(RecordReplay)
add_subdirectory(RVC2)
add_subdirectory(StereoDepth)
add_subdirectory(Script)
add_subdirectory(SpatialDetectionNetwork)
add_subdirectory(SpatialLocationCalculator)
add_subdirectory(Sync)
add_subdirectory(VideoEncoder)
add_subdirectory(Visualizer)
add_subdirectory(RGBD)
add_subdirectory(Warp)
add_subdirectory(ImageAlign)
add_subdirectory(NeuralDepth)
add_subdirectory(Vpp)
