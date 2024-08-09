#find_package(Protobuf REQUIRED)

set(MESSAGES_PROTOS ImgDetections.proto ImgFrame.proto SpatialImgDetections.proto)
set(MESSAGES_SRCS)
set(MESSAGES_HDRS)
set(PROTO_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/include/depthai/utility/protos)

#Code Generation
foreach(proto_file ${MESSAGES_PROTOS})
  get_filename_component(proto_file_abs ${proto_file} ABSOLUTE)
  get_filename_component(basename ${proto_file} NAME_WE)
  set(generated_files ${basename}.pb.cc ${basename}.pb.h)

  list(APPEND MESSAGES_SRCS ${basename}.pb.cc)
  list(APPEND MESSAGES_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${basename}.pb.h)

message("Generating files: ${generated_files}")

  add_custom_command(
      OUTPUT ${generated_files}
      COMMAND protobuf::protoc --cpp_out ${PROTO_OUTPUT_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} ${proto_file_abs}
      COMMENT "Generating ${generated_files} from ${proto_file}"
      VERBATIM
  )
endforeach()

add_library(libProto ${MESSAGES_SRCS})
target_link_libraries(libProto protobuf::libprotobuf)
#target_include_directories(libProto PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/..)
target_include_directories(libProto
  PUBLIC
    #${CMAKE_CURRENT_BINARY_DIR}/..
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
)
