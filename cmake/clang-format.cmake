# Copyright Tomas Zeman 2019.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

function(clangformat_setup)
  if(NOT CLANGFORMAT_EXECUTABLE)
    set(CLANGFORMAT_EXECUTABLE clang-format)
  endif()

  if(NOT EXISTS ${CLANGFORMAT_EXECUTABLE})
    find_program(clangformat_executable_tmp ${CLANGFORMAT_EXECUTABLE})
    if(clangformat_executable_tmp)
      set(CLANGFORMAT_EXECUTABLE ${clangformat_executable_tmp})
      unset(clangformat_executable_tmp)
    else()
      message(STATUS "ClangFormat: ${CLANGFORMAT_EXECUTABLE} not found! Target 'clangformat' not available...")
      return()
    endif()
  endif()

  foreach(clangformat_source ${ARGV})
    get_filename_component(clangformat_source ${clangformat_source} ABSOLUTE)
    list(APPEND clangformat_sources ${clangformat_source})
  endforeach()

  add_custom_target(${PROJECT_NAME}_clangformat
    COMMAND
      ${CLANGFORMAT_EXECUTABLE}
      -style=file
      -i
      ${clangformat_sources}
    WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR}
    COMMENT
      "Formating with ${CLANGFORMAT_EXECUTABLE} ..."
  )

  if(TARGET clangformat)
    add_dependencies(clangformat ${PROJECT_NAME}_clangformat)
  else()
    add_custom_target(clangformat DEPENDS ${PROJECT_NAME}_clangformat)
  endif()
endfunction()


macro(header_directories header_dirs return_list)
  
  foreach(header_dir ${header_dirs})
    file(GLOB_RECURSE new_list "${header_dir}/*.hpp")
    set(file_list "")
    foreach(file_path ${new_list})
      set(file_list "${file_list}" ${file_path})
    endforeach()
    list(REMOVE_DUPLICATES file_list)
    set(${return_list} "${${return_list}}" "${file_list}")
  endforeach()

endmacro()

function(target_clangformat_setup target header_dirs)
  get_target_property(target_sources ${target} SOURCES)
  header_directories("${header_dirs}" header_files)
  set(target_files_to_format "${target_sources};${header_files}")
  clangformat_setup(${target_files_to_format})
endfunction()
