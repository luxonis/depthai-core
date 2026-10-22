cmake_minimum_required(VERSION 3.20)

# Expand each original argument in quotes, without converting it to a CMake list.
math(EXPR lastArgument "${CMAKE_ARGC} - 1")
foreach(_arg RANGE ${lastArgument})
    if(DEFINED commandStart AND _arg GREATER_EQUAL commandStart)
        string(APPEND commandArguments " \"\${CMAKE_ARGV${_arg}}\"")
        string(APPEND arguments " [${CMAKE_ARGV${_arg}}]")
    elseif(NOT DEFINED commandStart AND "${CMAKE_ARGV${_arg}}" STREQUAL "-P")
        math(EXPR commandStart "${_arg} + 2")
    endif()
endforeach()

message(STATUS "arguments: ${arguments}")

# Check if ENV variable TEST_TIMEOUT is set and use that rather than TIMEOUT_SECONDS
if(DEFINED ENV{TEST_TIMEOUT} AND NOT DEFINED FORCE_TIMEOUT_SECONDS)
    message(STATUS "Overriding timeout: ${TIMEOUT_SECONDS} with $ENV{TEST_TIMEOUT}")
    set(TIMEOUT_SECONDS $ENV{TEST_TIMEOUT})
endif()

# Check if FORCE_TIMEOUT_SECONDS is set, in that case respect that option
if(FORCE_TIMEOUT_SECONDS)
    message(STATUS "Forcing timeout of ${FORCE_TIMEOUT_SECONDS} seconds")
    set(TIMEOUT_SECONDS ${FORCE_TIMEOUT_SECONDS})
endif()

# Execute the example (SIGTERM for now, could be improved with SIGINT -> SIGKILL)
if(TIMEOUT_SECONDS GREATER 0)
    cmake_language(EVAL CODE "execute_process(COMMAND ${commandArguments} TIMEOUT \"\${TIMEOUT_SECONDS}\" RESULT_VARIABLE error_variable)")
else()
    cmake_language(EVAL CODE "execute_process(COMMAND ${commandArguments} RESULT_VARIABLE error_variable)")
endif()

message(STATUS "After process executed, ${PATH_TO_TEST_EXECUTABLE} produced the following exit code: ${error_variable}")

if(error_variable MATCHES "timeout" OR error_variable EQUAL 128 OR error_variable EQUAL 124 OR error_variable EQUAL 9)
    # Okay
elseif(NOT error_variable)
    # return code == 0, also okay
elseif(error_variable EQUAL 125)
    message(STATUS "No device available, ${PATH_TO_TEST_EXECUTABLE} skipped")
elseif(error_variable EQUAL 133 OR error_variable MATCHES "Child killed")
    # sigkill, return fatal error but mark the issue
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} had to be forcefully killed after 5 additional seconds after SIGINT")
else()
    # not timeout and error code != 0, not okay
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} produced an error (${error_variable}) while running")
endif()

