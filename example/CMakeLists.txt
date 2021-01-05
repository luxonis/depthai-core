cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -pedantic -O3 -std=c++11 ")

# Add depthai-cpp dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/depthai-core EXCLUDE_FROM_ALL)

set(TARGET_NAME myapp)
project(${TARGET_NAME})

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add source files
add_executable("${TARGET_NAME}" 
    main.cpp
)

# Link with libraries
target_link_libraries(${TARGET_NAME} 
    PUBLIC 
        depthai-core
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBS}
)
