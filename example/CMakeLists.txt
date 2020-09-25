cmake_minimum_required(VERSION 3.2)


include("${CMAKE_CURRENT_LIST_DIR}/../cmake/HunterGate.cmake")
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
    SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
    FILEPATH "${CMAKE_CURRENT_LIST_DIR}/../cmake/Hunter/config.cmake" # Local config for dependencies
)

# Add depthai-cpp dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/depthai-core EXCLUDE_FROM_ALL)

set(TARGET_NAME myapp)
project(${TARGET_NAME})


# Dependencies
find_package(OpenCV REQUIRED)

hunter_add_package(FP16)
find_package(FP16 REQUIRED)

# Add source files
add_executable("${TARGET_NAME}" 
    main.cpp
)

# Link with libraries
target_link_libraries(${TARGET_NAME} 
    PUBLIC 
        depthai-core
        ${OpenCV_LIBS}
        FP16::fp16
)
