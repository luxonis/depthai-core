if(NOT CLANG_TIDY_BIN)
  find_program(CLANG_TIDY_BIN clang-tidy)
endif()

if(NOT CLANG_TIDY_BIN)
  message(FATAL_ERROR "clang-tidy is not installed. Aborting...")
else()
  message(STATUS "clang-tidy has been found: ${CLANG_TIDY_BIN}")
endif()

set_target_properties(${TARGET_NAME} PROPERTIES C_CLANG_TIDY ${CLANG_TIDY_BIN} CXX_CLANG_TIDY ${CLANG_TIDY_BIN})
