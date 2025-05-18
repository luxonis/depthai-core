cmake_minimum_required(VERSION 3.10)

# Find Protobuf package
find_package(Protobuf REQUIRED)

# Print out where Protobuf was found
message(STATUS "Found Protobuf: ${Protobuf_DIR}")

# Get the directory containing the CMakeLists.txt file
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Find all .proto files in the same directory as CMakeLists.txt
file(GLOB MESSAGES_PROTOS "${PROTO_DIR}/*.proto")

# Initialize lists for sources and headers
set(MESSAGES_SRCS)
set(MESSAGES_HDRS)

# Set output directory for generated files
set(PROTO_GEN_DIR ${CMAKE_BINARY_DIR}/include/depthai/schemas)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})


# Code Generation
foreach(proto_file ${MESSAGES_PROTOS})
    get_filename_component(basename ${proto_file} NAME_WE)
    set(generated_files ${PROTO_GEN_DIR}/${basename}.pb.cc ${PROTO_GEN_DIR}/${basename}.pb.h)

    list(APPEND MESSAGES_SRCS ${PROTO_GEN_DIR}/${basename}.pb.cc)
    list(APPEND MESSAGES_HDRS ${PROTO_GEN_DIR}/${basename}.pb.h)

    add_custom_command(
        OUTPUT ${generated_files}
        COMMAND protobuf::protoc --cpp_out=${PROTO_GEN_DIR} -I ${PROTO_DIR} ${proto_file}
        DEPENDS ${proto_file}
        COMMENT "Generating ${generated_files} from ${proto_file}"
        VERBATIM
    )
endforeach()

# Create a library target for the generated protobuf files
add_library(messages STATIC ${MESSAGES_SRCS})

# Link the protobuf library
target_link_libraries(messages protobuf::libprotobuf)

# Include the directory with generated headers
# Use BUILD_INTERFACE and INSTALL_INTERFACE to properly manage include directories
target_include_directories(messages PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/>
    $<INSTALL_INTERFACE:include/schemas>
)

