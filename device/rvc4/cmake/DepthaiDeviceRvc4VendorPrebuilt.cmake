# Prebuilt private module (depthai-device-rvc4-vendor, the vendor module) and prebuilt Hexagon DSP offload.
#
# A build with the sources of the private module (the Luxonis CI) exports them as one archive:
#   lib/libdepthai-device-rvc4-vendor.a   the base archive (the ToF wrapper is a separate archive and is not exported)
#   PREBUILT.json                  version, commits, sanitizer, vcpkg triplet, compiler, sysroot, interface defines,
#                                  link interface
#   PREBUILT.headers               "<sha256>  <root>/<path>" for every header that the base archive compiled against;
#                                  root is source (this tree), core (depthai-core) or build (the build directory)
#   dsp/lib/                       libImageAlignDsp.a, libStereoDepthDsp.a, libImageAlignDsp_skel.so,
#                                  libStereoDepthDsp_skel.so, when the build had the Hexagon SDK
#   dsp/PREBUILT.headers           the same for the DSP offload, plus every file of src/hexagon
# A build without the sources uses such an archive only when it compiles against the same headers and defines.

include_guard(GLOBAL)
set(_DEPTHAI_DEVICE_RVC4_VENDOR_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(_DEPTHAI_DEVICE_DSP_FILES libImageAlignDsp.a libStereoDepthDsp.a libImageAlignDsp_skel.so libStereoDepthDsp_skel.so)

# The part of this build that a prebuilt archive must match. Sets in the caller's scope:
#   DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER  none, asan-ubsan or tsan
#   DEPTHAI_DEVICE_RVC4_VENDOR_COMPILER   compiler id and major version (the C++ ABI of the archive)
#   DEPTHAI_DEVICE_RVC4_VENDOR_SYSROOT    Luxonis OS release of the sysroot (ID, VERSION_ID and MANIFEST_REV_REV of os-release)
function(depthai_device_rvc4_vendor_identity)
    if(DEPTHAI_TSAN)
        set(_sanitizer "tsan")
    elseif(DEPTHAI_ASAN_UBSAN)
        set(_sanitizer "asan-ubsan")
    else()
        set(_sanitizer "none")
    endif()
    string(REGEX MATCH "^[0-9]+" _major "${CMAKE_CXX_COMPILER_VERSION}")

    set(_sysroot "unknown")
    foreach(_os_release etc/os-release usr/lib/os-release)
        if(EXISTS "${DEPTHAI_DEVICE_RVC4_SYSROOT}/${_os_release}")
            set(_fields "")
            foreach(_key ID VERSION_ID MANIFEST_REV_REV)
                file(STRINGS "${DEPTHAI_DEVICE_RVC4_SYSROOT}/${_os_release}" _line REGEX "^${_key}=")
                string(REGEX REPLACE "^${_key}=\"?([^\"]*)\"?$" "\\1" _value "${_line}")
                list(APPEND _fields "${_value}")
            endforeach()
            string(JOIN " " _sysroot ${_fields})
            break()
        endif()
    endforeach()

    set(DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER "${_sanitizer}" PARENT_SCOPE)
    set(DEPTHAI_DEVICE_RVC4_VENDOR_COMPILER "${CMAKE_CXX_COMPILER_ID} ${_major}" PARENT_SCOPE)
    set(DEPTHAI_DEVICE_RVC4_VENDOR_SYSROOT "${_sysroot}" PARENT_SCOPE)
endfunction()

# Build with the sources of the private module (and so with the Hexagon SDK): writes the input of the export script of
# the private repository to <dir>: build.json (identity, link interface of the base archive, the roots),
# archive-<config>.txt (the base archive) and dsp-<config>.txt (the four files of the DSP offload).
function(depthai_device_rvc4_vendor_prebuilt_describe dir)
    depthai_device_rvc4_vendor_identity()
    set(_json "{}")
    foreach(_field "sanitizer;${DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER}" "triplet;${VCPKG_TARGET_TRIPLET}"
                   "compiler;${DEPTHAI_DEVICE_RVC4_VENDOR_COMPILER}" "compiler_version;${CMAKE_CXX_COMPILER_VERSION}"
                   "sysroot;${DEPTHAI_DEVICE_RVC4_VENDOR_SYSROOT}" "source_dir;${DEPTHAI_DEVICE_SOURCE_DIR}"
                   "core_dir;${DEPTHAI_DEVICE_KB_CORE_PATH}" "binary_dir;${DEPTHAI_DEVICE_BINARY_DIR}"
                   "vendor_source_dir;${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}")
        list(GET _field 0 _key)
        list(GET _field 1 _value)
        string(REPLACE "\\" "\\\\" _value "${_value}")
        string(REPLACE "\"" "\\\"" _value "${_value}")
        string(JSON _json SET "${_json}" "${_key}" "\"${_value}\"")
    endforeach()
    # Link interface of the base archive as the private module declares it ($<LINK_ONLY:...> kept as text)
    get_target_property(_link depthai-device-rvc4-vendor INTERFACE_LINK_LIBRARIES)
    set(_array "[]")
    set(_index 0)
    foreach(_item IN LISTS _link)
        string(REPLACE "\\" "\\\\" _item "${_item}")
        string(REPLACE "\"" "\\\"" _item "${_item}")
        string(JSON _array SET "${_array}" ${_index} "\"${_item}\"")
        math(EXPR _index "${_index} + 1")
    endforeach()
    string(JSON _json SET "${_json}" link_interface "${_array}")
    file(WRITE "${dir}/build.json" "${_json}\n")
    file(GENERATE OUTPUT "${dir}/archive-$<CONFIG>.txt" CONTENT "$<TARGET_FILE:depthai-device-rvc4-vendor>\n")
    string(JOIN "\n" _skels ${DEPTHAI_DEVICE_DSP_SKELS})
    file(GENERATE OUTPUT "${dir}/dsp-$<CONFIG>.txt"
         CONTENT "$<TARGET_FILE:ImageAlignDsp>\n$<TARGET_FILE:StereoDepthDsp>\n${_skels}\n")
    # The ToF part: its static libraries in link order (tof-<config>.txt) and its other link items (tof-link.txt)
    file(REMOVE "${dir}/tof-link.txt")
    if(TARGET depthai-device-rvc4-vendor-tof)
        _depthai_device_rvc4_vendor_link_closure(depthai-device-rvc4-vendor-tof _archives _others)
        set(_content "")
        foreach(_archive IN LISTS _archives)
            string(APPEND _content "$<TARGET_FILE:${_archive}>\n")
        endforeach()
        file(GENERATE OUTPUT "${dir}/tof-$<CONFIG>.txt" CONTENT "${_content}")
        string(JOIN "\n" _others ${_others})
        file(WRITE "${dir}/tof-link.txt" "${_others}\n")
    endif()
endfunction()

# The static libraries that <target> needs, in link order (<target> first; static library targets of this build), and
# the other link items (imported and interface targets, system libraries). The vendor base archive and the C interface
# target are left out: the import adds them. An absolute path of a system library becomes its name (liblog.so: log),
# because a prebuilt archive is used with another sysroot path.
function(_depthai_device_rvc4_vendor_link_closure target archives_var others_var)
    set(_archives "")
    set(_others "")
    set(_seen "")
    set(_queue "${target}")
    while(_queue)
        list(GET _queue 0 _item)
        list(REMOVE_AT _queue 0)
        if(_item MATCHES "^\\$<LINK_ONLY:(.+)>$")
            set(_item "${CMAKE_MATCH_1}")
        endif()
        if(TARGET "${_item}")
            get_target_property(_aliased "${_item}" ALIASED_TARGET)
            if(_aliased)
                set(_item "${_aliased}")
            endif()
        endif()
        if(_item IN_LIST _seen OR _item STREQUAL "" OR _item MATCHES "^\\$<"
           OR _item STREQUAL "depthai-device-rvc4-vendor" OR _item STREQUAL "depthai-device-vendor-api")
            continue()
        endif()
        list(APPEND _seen "${_item}")
        if(TARGET "${_item}")
            get_target_property(_type "${_item}" TYPE)
            get_target_property(_imported "${_item}" IMPORTED)
            if(_type STREQUAL "STATIC_LIBRARY" AND NOT _imported)
                list(APPEND _archives "${_item}")
                get_target_property(_deps "${_item}" LINK_LIBRARIES)
                if(_deps)
                    list(APPEND _queue ${_deps})
                endif()
            else()
                list(APPEND _others "${_item}")
            endif()
        elseif(IS_ABSOLUTE "${_item}" AND _item MATCHES "/lib([^/]+)\\.(so|a)$")
            list(APPEND _others "${CMAKE_MATCH_1}")
        else()
            list(APPEND _others "${_item}")
        endif()
    endwhile()
    set(${archives_var} "${_archives}" PARENT_SCOPE)
    set(${others_var} "${_others}" PARENT_SCOPE)
endfunction()

# Value of a field of PREBUILT.json (a string, or the elements of an array as a list); fails when it is missing.
function(_depthai_device_rvc4_vendor_prebuilt_field json key out_var)
    string(JSON _type ERROR_VARIABLE _error TYPE "${json}" "${key}")
    if(_error)
        message(FATAL_ERROR "Prebuilt private module: PREBUILT.json has no \"${key}\"")
    endif()
    if(_type STREQUAL "ARRAY")
        string(JSON _count LENGTH "${json}" "${key}")
        set(_value "")
        if(_count GREATER 0)
            math(EXPR _last "${_count} - 1")
            foreach(_index RANGE ${_last})
                string(JSON _item GET "${json}" "${key}" ${_index})
                list(APPEND _value "${_item}")
            endforeach()
        endif()
    else()
        string(JSON _value GET "${json}" "${key}")
    endif()
    set(${out_var} "${_value}" PARENT_SCOPE)
endfunction()

# Existing files of a header list, as dependencies of the build-time check. A missing file is reported by the check.
function(_depthai_device_rvc4_vendor_prebuilt_list_files list out_var)
    file(STRINGS "${list}" _lines)
    set(_files "")
    foreach(_line IN LISTS _lines)
        if(_line MATCHES "^[0-9a-f]+  source/(.+)$")
            set(_file "${DEPTHAI_DEVICE_SOURCE_DIR}/${CMAKE_MATCH_1}")
        elseif(_line MATCHES "^[0-9a-f]+  core/(.+)$")
            set(_file "${DEPTHAI_DEVICE_KB_CORE_PATH}/${CMAKE_MATCH_1}")
        elseif(_line MATCHES "^[0-9a-f]+  build/(.+)$")
            set(_file "${DEPTHAI_DEVICE_BINARY_DIR}/${CMAKE_MATCH_1}")
        else()
            message(FATAL_ERROR "Prebuilt private module: malformed line in ${list}: '${_line}'")
        endif()
        if(EXISTS "${_file}")
            list(APPEND _files "${_file}")
        endif()
    endforeach()
    set(${out_var} "${_files}" PARENT_SCOPE)
endfunction()

# Build without the sources of the private module: imports the base archive of <prebuilt_dir> as the target
# depthai-device-rvc4-vendor and checks it. The configure step stops when the archive was built for another sanitizer, vcpkg
# triplet, compiler or sysroot, or against other headers of the C interface. The target
# depthai-device-rvc4-vendor-prebuilt-check checks the headers again before the code that uses the archive compiles, and
# after each change of a listed header. A tof/ folder in the archive gives the target depthai-device-rvc4-vendor-tof. With <use_dsp> the DSP offload of the archive is checked too.
function(depthai_device_rvc4_vendor_prebuilt_import prebuilt_dir work_dir use_dsp)
    foreach(_file lib/libdepthai-device-rvc4-vendor.a PREBUILT.json PREBUILT.headers)
        if(NOT EXISTS "${prebuilt_dir}/${_file}")
            message(FATAL_ERROR "Prebuilt private module: ${prebuilt_dir}/${_file} not found")
        endif()
    endforeach()
    file(READ "${prebuilt_dir}/PREBUILT.json" _json)
    _depthai_device_rvc4_vendor_prebuilt_field("${_json}" format _format)
    if(NOT _format EQUAL 2)
        message(FATAL_ERROR "Prebuilt private module: PREBUILT.json has format ${_format}; this tree reads format 2 (the C "
                            "interface vendor/dai_device_*.h). Use a vendor archive that was built for this interface.")
    endif()
    _depthai_device_rvc4_vendor_prebuilt_field("${_json}" version _version)

    depthai_device_rvc4_vendor_identity()
    set(_mismatches "")
    foreach(_check "sanitizer;${DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER}" "triplet;${VCPKG_TARGET_TRIPLET}"
                   "compiler;${DEPTHAI_DEVICE_RVC4_VENDOR_COMPILER}" "sysroot;${DEPTHAI_DEVICE_RVC4_VENDOR_SYSROOT}")
        list(GET _check 0 _key)
        list(GET _check 1 _expected)
        _depthai_device_rvc4_vendor_prebuilt_field("${_json}" "${_key}" _actual)
        if(NOT _actual STREQUAL _expected)
            string(APPEND _mismatches "\n  ${_key}: archive '${_actual}', this build '${_expected}'")
        endif()
    endforeach()
    if(_mismatches)
        message(FATAL_ERROR "The prebuilt private module ${_version} (${prebuilt_dir}) is not built for this build:${_mismatches}\n"
                            "Use the prebuilt archive of the same sanitizer variant (rvc4, rvc4-asan-ubsan, rvc4-tsan) and the "
                            "sysroot of the same Luxonis OS release.")
    endif()

    _depthai_device_rvc4_vendor_prebuilt_field("${_json}" link_interface _link_interface)
    add_library(depthai-device-rvc4-vendor STATIC IMPORTED GLOBAL)
    set_target_properties(depthai-device-rvc4-vendor PROPERTIES
        IMPORTED_LOCATION "${prebuilt_dir}/lib/libdepthai-device-rvc4-vendor.a"
        INTERFACE_LINK_LIBRARIES "${_link_interface}")

    set(_lists "${prebuilt_dir}/PREBUILT.headers")
    if(use_dsp)
        list(APPEND _lists "${prebuilt_dir}/dsp/PREBUILT.headers")
    endif()
    file(MAKE_DIRECTORY "${work_dir}")
    # The ToF part (tof/): its static libraries in link order, then the base archive and its other link items
    if(EXISTS "${prebuilt_dir}/tof/PREBUILT.libraries")
        file(STRINGS "${prebuilt_dir}/tof/PREBUILT.libraries" _tof_libraries)
        _depthai_device_rvc4_vendor_prebuilt_field("${_json}" tof_link_interface _tof_link_items)
        # Imported targets of the vendor build that this build may not have: Threads::Threads from find_package, the
        # others (for example unwind::unwind of glog) as the library of the sysroot with that name
        set(_tof_link "")
        foreach(_item IN LISTS _tof_link_items)
            if(_item STREQUAL "Threads::Threads" AND NOT TARGET Threads::Threads)
                find_package(Threads REQUIRED)
            elseif(_item MATCHES "^[^:]+::([^:]+)$" AND NOT TARGET "${_item}")
                set(_item "${CMAKE_MATCH_1}")
            endif()
            list(APPEND _tof_link "${_item}")
        endforeach()
        set(_tof_targets "")
        set(_index 0)
        foreach(_library IN LISTS _tof_libraries)
            if(NOT EXISTS "${prebuilt_dir}/tof/lib/${_library}")
                message(FATAL_ERROR "Prebuilt private module: ${prebuilt_dir}/tof/lib/${_library} not found")
            endif()
            add_library(depthai-device-rvc4-vendor-tof-part${_index} STATIC IMPORTED GLOBAL)
            set_target_properties(depthai-device-rvc4-vendor-tof-part${_index} PROPERTIES IMPORTED_LOCATION "${prebuilt_dir}/tof/lib/${_library}")
            list(APPEND _tof_targets depthai-device-rvc4-vendor-tof-part${_index})
            math(EXPR _index "${_index} + 1")
        endforeach()
        add_library(depthai-device-rvc4-vendor-tof INTERFACE IMPORTED GLOBAL)
        set_target_properties(depthai-device-rvc4-vendor-tof PROPERTIES
            INTERFACE_LINK_LIBRARIES "${_tof_targets};depthai-device-rvc4-vendor;${_tof_link}")
    endif()

    set(_check_args
        -D "PREBUILT_DIR=${prebuilt_dir}" -D "VERSION=${_version}" -D "HEADER_LISTS=${_lists}"
        -D "SOURCE_DIR=${DEPTHAI_DEVICE_SOURCE_DIR}" -D "CORE_DIR=${DEPTHAI_DEVICE_KB_CORE_PATH}"
        -D "BINARY_DIR=${DEPTHAI_DEVICE_BINARY_DIR}")
    # The defines are known only at generate time; the headers are checked now, so that a mismatch stops the configure
    execute_process(COMMAND "${CMAKE_COMMAND}" ${_check_args} -D CONFIGURE=ON
                            -P "${_DEPTHAI_DEVICE_RVC4_VENDOR_MODULE_DIR}/DepthaiDeviceRvc4VendorPrebuiltCheck.cmake"
                    RESULT_VARIABLE _result ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        string(REGEX REPLACE "^CMake Error at [^\n]*\n" "" _error "${_error}")
        message(FATAL_ERROR "${_error}")
    endif()

    set(_headers "")
    foreach(_list IN LISTS _lists)
        _depthai_device_rvc4_vendor_prebuilt_list_files("${_list}" _files)
        list(APPEND _headers ${_files})
    endforeach()
    set(_stamp "${work_dir}/checked-$<CONFIG>.stamp")
    add_custom_command(OUTPUT "${_stamp}"
        COMMAND "${CMAKE_COMMAND}" ${_check_args}
                -D "STAMP=${_stamp}"
                -P "${_DEPTHAI_DEVICE_RVC4_VENDOR_MODULE_DIR}/DepthaiDeviceRvc4VendorPrebuiltCheck.cmake"
        DEPENDS ${_lists} "${_DEPTHAI_DEVICE_RVC4_VENDOR_MODULE_DIR}/DepthaiDeviceRvc4VendorPrebuiltCheck.cmake" ${_headers}
        COMMENT "Checking the prebuilt private module against the C interface headers"
        VERBATIM)
    add_custom_target(depthai-device-rvc4-vendor-prebuilt-check DEPENDS "${_stamp}")
    set_property(DIRECTORY "${CMAKE_SOURCE_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${prebuilt_dir}/PREBUILT.json" ${_lists})
    list(LENGTH _headers _count)
    message(STATUS "Prebuilt private module: ${_version}, ${_count} files checked")
endfunction()

# Imports the prebuilt DSP offload of <dsp_dir> (the dsp/ folder of a prebuilt archive) as the targets ImageAlignDsp and
# StereoDepthDsp, and sets DEPTHAI_DEVICE_DSP_SKELS in the caller's scope to its two skeletons.
function(depthai_device_dsp_prebuilt_import dsp_dir)
    foreach(_file IN LISTS _DEPTHAI_DEVICE_DSP_FILES)
        if(NOT EXISTS "${dsp_dir}/lib/${_file}")
            message(FATAL_ERROR "The prebuilt Hexagon DSP offload in ${dsp_dir} has no lib/${_file}")
        endif()
    endforeach()
    foreach(_lib ImageAlignDsp StereoDepthDsp)
        add_library(${_lib} STATIC IMPORTED GLOBAL)
    endforeach()
    # ImageAlignDsp uses the common part of the DSP offload in StereoDepthDsp (log, status names)
    set_target_properties(ImageAlignDsp PROPERTIES
        IMPORTED_LOCATION "${dsp_dir}/lib/libImageAlignDsp.a"
        INTERFACE_LINK_LIBRARIES "StereoDepthDsp;cdsprpc")
    set_target_properties(StereoDepthDsp PROPERTIES
        IMPORTED_LOCATION "${dsp_dir}/lib/libStereoDepthDsp.a"
        INTERFACE_LINK_LIBRARIES "cdsprpc")
    set(DEPTHAI_DEVICE_DSP_SKELS "${dsp_dir}/lib/libImageAlignDsp_skel.so" "${dsp_dir}/lib/libStereoDepthDsp_skel.so" PARENT_SCOPE)
endfunction()

# Build without the sources: extracts the vendor archive <archive> (a local DEPTHAI_DEVICE_RVC4_VENDOR or the download of
# DEPTHAI_DEVICE_RVC4_VENDOR_VERSION) once into <work_dir>/extract/<start of its sha256>, and sets <out_var> to the folder
# in it that has PREBUILT.json. A new archive file runs the configure step again; older extractions are removed.
function(depthai_device_rvc4_vendor_extract archive work_dir out_var)
    file(SHA256 "${archive}" _hash)
    string(SUBSTRING "${_hash}" 0 16 _hash)
    set(_dir "${work_dir}/extract/${_hash}")
    file(GLOB _found "${_dir}/*/PREBUILT.json")
    if(NOT _found)
        file(REMOVE_RECURSE "${_dir}" "${_dir}.partial")
        file(ARCHIVE_EXTRACT INPUT "${archive}" DESTINATION "${_dir}.partial")
        file(RENAME "${_dir}.partial" "${_dir}")
        file(GLOB _found "${_dir}/*/PREBUILT.json")
    endif()
    list(LENGTH _found _count)
    if(NOT _count EQUAL 1)
        message(FATAL_ERROR "The vendor archive ${archive} must have one <folder>/PREBUILT.json (found ${_count}).")
    endif()
    file(GLOB _old LIST_DIRECTORIES true "${work_dir}/extract/*")
    list(REMOVE_ITEM _old "${_dir}")
    if(_old)
        file(REMOVE_RECURSE ${_old})
    endif()
    get_filename_component(_found "${_found}" DIRECTORY)
    set_property(DIRECTORY "${CMAKE_SOURCE_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${archive}")
    set(${out_var} "${_found}" PARENT_SCOPE)
endfunction()
