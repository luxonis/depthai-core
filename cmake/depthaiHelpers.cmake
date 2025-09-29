macro(add_runtime_dependencies depending_target dependency)
    if(TARGET ${dependency})
        get_property(imported_configs TARGET ${dependency} PROPERTY IMPORTED_CONFIGURATIONS)
        set(dlls "")
        message(DEBUG "Adding runtime dependencies for ${depending_target} on ${dependency}. Imported configurations: ${imported_configs}")
        foreach(cfg ${imported_configs})
            message(DEBUG "Adding runtime dependencies for ${depending_target} on ${dependency} (${cfg})")
            get_property(dll TARGET ${dependency} PROPERTY IMPORTED_LOCATION_${cfg})
            message(DEBUG "Retrieved dll for ${cfg}: '${dll}'")
            list(APPEND dlls $<$<CONFIG:${cfg}>:${dll}>)
        endforeach()
        message(DEBUG "Required dlls for ${depending_target} on ${dependency} are: ${dlls}")
    endif()
    # Create a list of required dll files
    set(required_dll_files ${dlls})
    # Copy the required dlls
    if(WIN32)
        add_custom_command(TARGET ${depending_target} POST_BUILD COMMAND
            "$<$<BOOL:${required_dll_files}>:${CMAKE_COMMAND};-E;copy_if_different;${required_dll_files};$<TARGET_FILE_DIR:${depending_target}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
        message(DEBUG "Required dlls for core are: ${required_dll_files}")
    endif()
endmacro()