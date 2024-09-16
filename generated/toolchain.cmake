# STEP 1: Lock variables

if(NOT "" STREQUAL "")
  set(ANDROID_ABI "")
endif()
if(NOT "" STREQUAL "")
  set(ANDROID_PLATFORM "")
endif()
if(NOT "" STREQUAL "")
  set(ANDROID_STL "")
endif()
if(NOT "" STREQUAL "")
  set(ANDROID_ARM_NEON "")
endif()
if(NOT "" STREQUAL "")
  set(ANDROID_ARM_MODE "")
endif()

# STEP 2: Include original toolchain (if exists)

if(NOT "" STREQUAL "")

  # Original toolchain file will be saved in variable:
  # * _INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE

  if("" STREQUAL "")
    # Variable not set, initial project configuration.
    # Path to original toolchain saved in CMAKE_TOOLCHAIN_FILE.
    if(NOT EXISTS "")
      message(
          FATAL_ERROR "File not found: ''"
      )
    endif()
    set(_original_toolchain_file "")
  else()
    set(_original_toolchain_file "")
  endif()

  set(
      _INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE
      "${_original_toolchain_file}"
      CACHE
      PATH
      "Original toolchain"
  )

  if("${_INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE}" STREQUAL "${CMAKE_CURRENT_LIST_FILE}")
    message(FATAL_ERROR "Internal error")
  endif()

  set(ANDROID_NDK "") # Suppress warning
  include("${_INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE}")

endif()


# STEP 3: Custom user variables

if(NOT "" STREQUAL "" OR NOT "@BUILD_PYTHON" STREQUAL "")
  # Simple PIC toolchain, which enables shared library building
  # This option is propagated to 3rdparty dependecies as well
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
