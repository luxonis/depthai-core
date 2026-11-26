# Generates the API documentation of the public headers with Doxygen (DEPTHAI_BUILD_DOCS=ON).
#
# - Target: doxygen (also part of ALL).
# - Output: html/ and xml/ in DEPTHAI_DOXYGEN_OUTPUT_DIR. Default: <build folder>/docs, the same folder as before the move
#   from docs/. Tools that generate the API reference read the XML there: keep the target name and this default.
# - Template: Doxyfile.in in this folder. configure_file() replaces the @VAR@ values.
#
# DepthaiDoxygen()

function(DepthaiDoxygen)
    # Get doxygen executable - required
    find_package(Doxygen REQUIRED)

    # options
    set(DEPTHAI_DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs" CACHE STRING "Output directory where doxygen should generate the documentation")

    # Retrieves all public header files for specified target
    include(target-public-headers)
    get_target_public_headers(${TARGET_CORE_NAME} header_files)

    # Creates a space separated list of files
    set(DOXYGEN_INPUT_FILES "")
    foreach(file ${header_files})
        set(DOXYGEN_INPUT_FILES "${DOXYGEN_INPUT_FILES} \"${file}\"")
    endforeach()

    # Doxygen variables
    set(DOXYGEN_OUTPUT_DIR ${DEPTHAI_DOXYGEN_OUTPUT_DIR})
    set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
    set(DOXYFILE_IN ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/Doxyfile.in)
    set(DOXYFILE_OUT ${DOXYGEN_OUTPUT_DIR}/Doxyfile)

    # Replace variables inside @@ with the current values
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

    # Create needed directories
    # Doxygen won't create this for us
    file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

    # Add command to run doxygen documentation
    # Doxygen runs in the folder of the template, as before the move: with an empty STRIP_FROM_PATH, Doxygen removes
    # the path of its working directory from the file names, and no input file is in this folder.
    add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                       DEPENDS "${header_files}"
                       COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
                       MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
                       COMMENT "Generating docs"
                       VERBATIM
                       COMMAND_EXPAND_LISTS)

    # Add a doxygen target
    add_custom_target(doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

    # Add property to target, output file
    set_target_properties(doxygen PROPERTIES DOXYGEN_OUTPUT_DIR ${DOXYGEN_OUTPUT_DIR})
endfunction()
