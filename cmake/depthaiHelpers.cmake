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
    if(DEFINED required_dll_files)
        list(APPEND required_dll_files ${dlls})
    else()
        set(required_dll_files ${dlls})
    endif()
    # Copy the required dlls
    if(WIN32)
        foreach(dll_path IN LISTS required_dll_files)
            add_custom_command(TARGET ${depending_target} POST_BUILD COMMAND
                ${CMAKE_COMMAND} -DDLLS=${dll_path} -DDEST_DIR=$<TARGET_FILE_DIR:${depending_target}>
                -P "${CMAKE_SOURCE_DIR}/cmake/CreateSymlinks.cmake"
                VERBATIM
            )
        endforeach()
        message(DEBUG "Required dlls for ${depending_target} are: ${required_dll_files}")
    endif()
endmacro()