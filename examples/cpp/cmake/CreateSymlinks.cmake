# CreateSymlinks.cmake - Creates symbolic links for DLLs
# Usage: cmake -DDLLS="dll1;dll2;..." -DDEST_DIR="path" -P CreateSymlinks.cmake

if(NOT DEFINED DLLS OR NOT DEFINED DEST_DIR)
    return()
endif()

foreach(dll IN LISTS DLLS)
    if(dll STREQUAL "")
        continue()
    endif()
    get_filename_component(dll_name "${dll}" NAME)
    set(link_path "${DEST_DIR}/${dll_name}")
    # Remove existing file/link if present
    if(EXISTS "${link_path}")
        file(REMOVE "${link_path}")
    endif()
    # Create symbolic link
    file(CREATE_LINK "${dll}" "${link_path}" SYMBOLIC RESULT result)
    if(NOT result EQUAL 0)
        message(WARNING "Failed to create symlink for ${dll_name}, falling back to copy")
        file(COPY "${dll}" DESTINATION "${DEST_DIR}")
    endif()
endforeach()
