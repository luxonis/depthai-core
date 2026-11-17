project(pipeline.node.IMU)

# The node talks to the sensor framework through the Luxonis SensorDirectChannel interface;
# the implementation (Qualcomm SEE direct channel client) lives in the private module.
set(HEADERS
    IMU.hpp
    SensorDirectChannel.hpp
)

set(SOURCES
    IMU.cpp
)

add_library(${PROJECT_NAME}
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
    PRIVATE
        "${DEPTHAI_DEVICE_KB_CORE_PATH}/include/"
        "${DEPTHAI_DEVICE_SOURCE_DIR}/src/utilities/"
        "${DEPTHAI_DEVICE_SOURCE_DIR}/src/"
        "${DEPTHAI_DEVICE_KB_CORE_PATH}/src/"
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        depthai::core
)
