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

# Add any custom binary data using the following snippet
## # Hunter test data download
## hunter_private_data(
##     URL "https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/depthai/depthai-0.0.2.1+06c6af4097c0156c7421611957682d4536995d5f-cp35-cp35m-macosx_10_9_x86_64.whl"
##     SHA1 "54dc11a757107afde494d64ea1900c5a0b9bb4a4"
##     FILE "test1.data"
##     LOCATION test1_data
## )
## message(STATUS "Location of test1.data: ${test1_data}")


# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)

