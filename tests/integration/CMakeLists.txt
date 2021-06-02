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

# Create target dependant on depthai::core
add_executable(integration-test-core src/main.cpp)
target_link_libraries(integration-test-core PRIVATE depthai::core)
# Set compiler standard (C++14)
set_property(TARGET integration-test-core PROPERTY CXX_STANDARD 14)
set_property(TARGET integration-test-core PROPERTY CXX_STANDARD_REQUIRED ON)


# Create target dependant on depthai::opencv
add_executable(integration-test-opencv src/main.cpp)
target_link_libraries(integration-test-opencv PRIVATE depthai::opencv)
# Set compiler standard (C++14)
set_property(TARGET integration-test-opencv PROPERTY CXX_STANDARD 14)
set_property(TARGET integration-test-opencv PROPERTY CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()
add_test(integration-test-core integration-test-core)
add_test(integration-test-opencv integration-test-opencv)