function(target_clangtidy_setup target)

    if(NOT CLANG_TIDY_BIN)
        find_program(CLANG_TIDY_BIN clang-tidy)
    endif()

    if(NOT CLANG_TIDY_BIN)
        message(FATAL_ERROR "clang-tidy is not installed. Aborting...")
    else()
        message(STATUS "clang-tidy has been found: ${CLANG_TIDY_BIN}")
    endif()

    # Disable for C compiler for now
    #set_target_properties(${target} PROPERTIES C_CLANG_TIDY ${CLANG_TIDY_BIN} CXX_CLANG_TIDY ${CLANG_TIDY_BIN})
    set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY ${CLANG_TIDY_BIN})

endfunction()