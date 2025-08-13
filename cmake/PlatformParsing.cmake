function(detect_platform_arch OUT_VAR)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc)
    set(_result "Unknown")

    if (WIN32 AND (_proc STREQUAL "amd64" OR _proc STREQUAL "x86_64"))
        set(_result "windows-x86_64")
    elseif (UNIX AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND (_proc STREQUAL "x86_64"))
        set(_result "linux-x86_64")
    elseif (UNIX AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND (_proc STREQUAL "aarch64" OR _proc STREQUAL "arm64"))
        set(_result "linux-arm64")
    elseif (APPLE AND (_proc STREQUAL "x86_64"))
        set(_result "macos-13-native") # TODO Rename this
    elseif (APPLE AND (_proc STREQUAL "arm64" OR _proc STREQUAL "aarch64"))
        set(_result "macos-14-native") # TODO Rename this
    endif()

    set(${OUT_VAR} "${_result}" PARENT_SCOPE)
endfunction()