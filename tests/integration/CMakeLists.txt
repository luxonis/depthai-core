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

# Helper function
function(add_runtime_dependencies depending_target dependency)
    if(WIN32)
        get_property(imported_configs TARGET ${dependency} PROPERTY IMPORTED_CONFIGURATIONS)
        set(dlls "")
        foreach(cfg ${imported_configs})
            get_property(dll TARGET ${dependency} PROPERTY IMPORTED_LOCATION_${cfg})
            set(dlls ${dlls} $<$<CONFIG:${cfg}>:${dll}>)
        endforeach()
        add_custom_command(TARGET ${depending_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${dlls}
            $<TARGET_FILE_DIR:${depending_target}>)
    endif()
endfunction()

# Create target dependant on depthai::core
add_executable(integration-test-core src/main.cpp)
target_link_libraries(integration-test-core PRIVATE depthai::core)

# Create target dependant on depthai::opencv
add_executable(integration-test-opencv src/main.cpp)
target_link_libraries(integration-test-opencv PRIVATE depthai::opencv)

# Add runtime dependencies
if(TEST_FIND_PACKAGE)
    add_runtime_dependencies(integration-test-core depthai::core)
    add_runtime_dependencies(integration-test-opencv depthai::opencv)
endif()

include(CTest)
enable_testing()
add_test(integration-test-core integration-test-core)
add_test(integration-test-opencv integration-test-opencv)