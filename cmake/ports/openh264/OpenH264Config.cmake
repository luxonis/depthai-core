include(CMakeFindDependencyMacro)
find_dependency(Threads)

get_filename_component(_openh264_prefix "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
find_path(OpenH264_INCLUDE_DIR NAMES wels/codec_api.h
    PATHS "${_openh264_prefix}/include" NO_DEFAULT_PATH)
find_library(OpenH264_LIBRARY_RELEASE NAMES openh264
    PATHS "${_openh264_prefix}/lib" NO_DEFAULT_PATH)
find_library(OpenH264_LIBRARY_DEBUG NAMES openh264d openh264
    PATHS "${_openh264_prefix}/debug/lib" NO_DEFAULT_PATH)
unset(_openh264_prefix)
include(SelectLibraryConfigurations)
select_library_configurations(OpenH264)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenH264 REQUIRED_VARS OpenH264_LIBRARY OpenH264_INCLUDE_DIR)
if(OpenH264_FOUND AND NOT TARGET OpenH264::OpenH264)
    add_library(OpenH264::OpenH264 UNKNOWN IMPORTED)
    set_target_properties(OpenH264::OpenH264 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OpenH264_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES Threads::Threads
        IMPORTED_LOCATION "${OpenH264_LIBRARY_RELEASE}"
    )
    if(OpenH264_LIBRARY_RELEASE)
        set_property(TARGET OpenH264::OpenH264 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(OpenH264::OpenH264 PROPERTIES IMPORTED_LOCATION_RELEASE "${OpenH264_LIBRARY_RELEASE}")
    endif()
    if(OpenH264_LIBRARY_DEBUG)
        set_property(TARGET OpenH264::OpenH264 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
        set_target_properties(OpenH264::OpenH264 PROPERTIES IMPORTED_LOCATION_DEBUG "${OpenH264_LIBRARY_DEBUG}")
        if(NOT OpenH264_LIBRARY_RELEASE)
            set_target_properties(OpenH264::OpenH264 PROPERTIES IMPORTED_LOCATION "${OpenH264_LIBRARY_DEBUG}")
        endif()
    endif()
endif()
mark_as_advanced(OpenH264_INCLUDE_DIR OpenH264_LIBRARY_RELEASE OpenH264_LIBRARY_DEBUG)
