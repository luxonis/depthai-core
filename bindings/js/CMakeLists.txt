set(TARGET_NAME depthai-js)

add_executable("${TARGET_NAME}" bindings.cpp)

add_dependencies("${TARGET_NAME}" "${PROJECT_NAME}-core")

target_link_options(
  "${TARGET_NAME}" 
  PRIVATE
  -fwasm-exceptions
  --embind-emit-tsd depthai-js.d.ts
  -sUSE_ES6_IMPORT_META=0
  -sENVIRONMENT=web
  -sMODULARIZE
  -sEXPORT_ES6
  -sEXPORT_NAME=depthaiJsModuleInitialize
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
