set(TARGET_NAME depthai-js)

add_executable("${TARGET_NAME}" bindings.cpp)

add_dependencies("${TARGET_NAME}" "${PROJECT_NAME}-core")

target_link_options(
  "${TARGET_NAME}" 
  PRIVATE
  -fwasm-exceptions
  --embind-emit-tsd depthai-js.d.ts
)
target_link_libraries(
  "${TARGET_NAME}"
  PRIVATE
  "${PROJECT_NAME}-core"
  -lembind
)
target_compile_options(
  "${TARGET_NAME}"
  PUBLIC
  -fwasm-exceptions
)
