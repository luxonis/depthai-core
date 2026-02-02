set(_internal_flags_sanitizer_list
    -fno-omit-frame-pointer
    -fno-optimize-sibling-calls
    -fsanitize=address,undefined,leak
    -fsanitize-address-use-after-scope
    -fsanitize=float-divide-by-zero
    -fno-sanitize=pointer-overflow
    -fno-sanitize-recover=all
)
string(JOIN " " _internal_flags_sanitizer ${_internal_flags_sanitizer_list})
set(CMAKE_C_FLAGS ${_internal_flags_sanitizer})
set(CMAKE_CXX_FLAGS ${_internal_flags_sanitizer})
set(CMAKE_C_FLAGS_INIT ${_internal_flags_sanitizer})
set(CMAKE_CXX_FLAGS_INIT ${_internal_flags_sanitizer})
set(CMAKE_EXE_LINKER_FLAGS ${_internal_flags_sanitizer})
set(CMAKE_EXE_LINKER_FLAGS_INIT ${_internal_flags_sanitizer})
set(CMAKE_SHARED_LINKER_FLAGS ${_internal_flags_sanitizer})
set(CMAKE_SHARED_LINKER_FLAGS_INIT ${_internal_flags_sanitizer})
set(CMAKE_MODULE_LINKER_FLAGS ${_internal_flags_sanitizer})
set(CMAKE_MODULE_LINKER_FLAGS_INIT ${_internal_flags_sanitizer})
set(DEPTHAI_SANITIZE ON)
set(SANITIZE_ADDRESS ON)
set(SANITIZE_UNDEFINED ON)
set(SANITIZE_THREAD OFF)
set(_internal_flags_sanitizer_list)
set(_internal_flags_sanitizer)
