include(CMakeFindDependencyMacro)
find_dependency(Threads)

get_filename_component(_openh264_prefix "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
find_path(OpenH264_INCLUDE_DIR NAMES wels/codec_api.h
    PATHS "${_openh264_prefix}/include" NO_DEFAULT_PATH)
find_library(OpenH264_LIBRARY_RELEASE NAMES openh264
    PATHS "${_openh264_prefix}/lib" NO_DEFAULT_PATH)
find_library(OpenH264_LIBRARY_DEBUG NAMES openh264d openh264
    PATHS "${_openh264_prefix}/debug/lib" NO_DEFAULT_PATH)
include(SelectLibraryConfigurations)
select_library_configurations(OpenH264)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenH264 REQUIRED_VARS OpenH264_LIBRARY OpenH264_INCLUDE_DIR)
if(OpenH264_FOUND AND NOT TARGET OpenH264::OpenH264)
    if("@VCPKG_LIBRARY_LINKAGE@" STREQUAL "dynamic")
        add_library(OpenH264::OpenH264 SHARED IMPORTED)
    else()
        add_library(OpenH264::OpenH264 STATIC IMPORTED)
    endif()
    set_target_properties(OpenH264::OpenH264 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OpenH264_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES Threads::Threads
    )
    foreach(_openh264_config RELEASE DEBUG)
        if(OpenH264_LIBRARY_${_openh264_config})
            set_property(TARGET OpenH264::OpenH264 APPEND PROPERTY IMPORTED_CONFIGURATIONS ${_openh264_config})
            if(WIN32 AND "@VCPKG_LIBRARY_LINKAGE@" STREQUAL "dynamic")
                if(_openh264_config STREQUAL "DEBUG")
                    set(_openh264_bin "${_openh264_prefix}/debug/bin")
                else()
                    set(_openh264_bin "${_openh264_prefix}/bin")
                endif()
                # Meson may include the ABI version in the DLL filename.
                file(GLOB _openh264_dlls RELATIVE "${_openh264_bin}" "${_openh264_bin}/*openh264*.dll")
                find_file(OpenH264_DLL_${_openh264_config}
                    NAMES openh264.dll openh264d.dll ${_openh264_dlls}
                    PATHS "${_openh264_bin}" NO_DEFAULT_PATH REQUIRED)
                set_target_properties(OpenH264::OpenH264 PROPERTIES
                    IMPORTED_IMPLIB_${_openh264_config} "${OpenH264_LIBRARY_${_openh264_config}}"
                    IMPORTED_LOCATION_${_openh264_config} "${OpenH264_DLL_${_openh264_config}}"
                )
                mark_as_advanced(OpenH264_DLL_${_openh264_config})
            else()
                set_target_properties(OpenH264::OpenH264 PROPERTIES
                    IMPORTED_LOCATION_${_openh264_config} "${OpenH264_LIBRARY_${_openh264_config}}")
            endif()
        endif()
    endforeach()
endif()
unset(_openh264_prefix)
unset(_openh264_config)
unset(_openh264_bin)
unset(_openh264_dlls)
mark_as_advanced(OpenH264_INCLUDE_DIR OpenH264_LIBRARY_RELEASE OpenH264_LIBRARY_DEBUG)
