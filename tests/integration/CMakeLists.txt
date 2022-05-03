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
        if(TARGET ${dependency})
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
        else()
            # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
            if(CMAKE_VERSION VERSION_LESS "3.21")
                file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
            else()
                set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${depending_target}>")
            endif()
            add_custom_command(TARGET ${depending_target} POST_BUILD COMMAND
                ${CMAKE_COMMAND} -E copy ${depthai_dll_libraries} $<TARGET_FILE_DIR:${depending_target}>
                COMMAND_EXPAND_LISTS
            )
        endif()
    endif()
endfunction()

# Create target dependant on depthai::core
add_executable(integration-test-core src/main.cpp)
target_link_libraries(integration-test-core PRIVATE depthai::core)

# Create target dependant on depthai::opencv
add_executable(integration-test-opencv src/main.cpp)
target_link_libraries(integration-test-opencv PRIVATE depthai::opencv)

# Add runtime dependencies
add_runtime_dependencies(integration-test-core usb-1.0)
add_runtime_dependencies(integration-test-opencv usb-1.0)

include(CTest)
enable_testing()
add_test(integration-test-core integration-test-core)
add_test(integration-test-opencv integration-test-opencv)