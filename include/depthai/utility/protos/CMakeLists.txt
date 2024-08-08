set(MESSAGES_PROTOS ImgDetections.proto ImgFrame.proto SpatialImgDetections.proto)
set(MESSAGES_SRCS)
set(MESSAGES_HDRS)

#Code Generation
foreach(proto_file ${MESSAGES_PROTOS})
  get_filename_component(proto_file_abs ${proto_file} ABSOLUTE)
  get_filename_component(basename ${proto_file} NAME_WE)
  set(generated_files ${basename}.pb.cc ${basename}.pb.h)

  #list(APPEND MESSAGES_SRCS ${basename}.pb.cc)
  list(APPEND MESSAGES_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${basename}.pb.cc)
  list(APPEND MESSAGES_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${basename}.pb.h)

  add_custom_command(
      OUTPUT ${generated_files}
      COMMAND protobuf::protoc --cpp_out ${CMAKE_CURRENT_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} ${proto_file_abs}
      DEPENDS ${proto_file}
      COMMENT "Generating ${generated_files} from ${proto_file}"
      VERBATIM
  )
endforeach()

add_library(messages_proto ${MESSAGES_SRCS})

target_link_libraries(messages_proto protobuf::libprotobuf)

#target_include_directories(messages_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/..)
target_include_directories(messages_proto PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
      $<INSTALL_INTERFACE:include>
  )

# Export and install the target
install(TARGETS messages_proto
    EXPORT messages_protoTargets
    DESTINATION lib/cmake/messages_proto
)

install(EXPORT messages_protoTargets
    FILE messages_protoTargets.cmake
    DESTINATION lib/cmake/messages_proto
)