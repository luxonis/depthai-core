# CreateSymlinks.cmake - Creates symbolic links for DLLs
# Usage: cmake -DDLLS="dll1;dll2;..." -DDEST_DIR="path" -P CreateSymlinks.cmake

message(DEBUG "Creating symlinks: ${DLLS} \n Destination path: ${DEST_DIR}")

if(NOT DEFINED DLLS OR NOT DEFINED DEST_DIR)
    return()
endif()

foreach(dll IN LISTS DLLS)
    # Extract dll_name (without path)
    get_filename_component(dll_name "${dll}" NAME)

    # Create destination
    set(link_path "${DEST_DIR}/${dll_name}")

    # Remove original symlink if it exists
    if(EXISTS "${link_path}")
        file(REMOVE "${link_path}")
    endif()

    message(DEBUG "Creating symlink: ${link_path} ---> ${dll}")

    # Create symbolic link
    file(CREATE_LINK "${dll}" "${link_path}" SYMBOLIC RESULT result_var)

    if(result_var)
        message(WARNING "Failed to create symlink from '${dll}' to '${link_path}': ${result_var}")
    endif()

endforeach()
