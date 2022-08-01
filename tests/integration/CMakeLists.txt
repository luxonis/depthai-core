cmake_minimum_required(VERSION 3.4)

# Add depthai using add_subdirectory
if(NOT TEST_FIND_PACKAGE)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../" depthai-core EXCLUDE_FROM_ALL)
endif()

# Create a project with name 'myapp'
set(TARGET_NAME myapp)
project(integration-test)

# Add depthai using find_package
if(TEST_FIND_PACKAGE)
    find_package(depthai CONFIG REQUIRED)
endif()

include(CTest)
enable_testing()

# Create target dependant on depthai::core
add_executable(integration-test-core src/main.cpp)
target_link_libraries(integration-test-core PRIVATE depthai::core)
add_test(integration-test-core integration-test-core)

# Create target dependant on depthai::opencv (if available)
if(TARGET depthai::opencv)
    add_executable(integration-test-opencv src/main.cpp)
    target_link_libraries(integration-test-opencv PRIVATE depthai::opencv)
    add_test(integration-test-opencv integration-test-opencv)
elseif("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
    message(STATUS "Skipping DepthAI OpenCV integration test as its not readily available on Win32 architecture.")
else()
    # Make sure that DepthAI OpenCV support is avaible with x64
    message(FATAL_ERROR "Integration test assumes DepthAI OpenCV support to be available, which seems to be missing.")
endif()
