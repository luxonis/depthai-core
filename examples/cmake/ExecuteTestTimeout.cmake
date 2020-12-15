# TODO (themarpe) - cross platform timeout

# Execute the example
execute_process(COMMAND timeout -s SIGINT -k 5 ${TIMEOUT_SECONDS} ${PATH_TO_TEST_EXECUTABLE} RESULT_VARIABLE error_variable)
message(STATUS "After timeout, ${PATH_TO_TEST_EXECUTABLE} produced the following exit code: ${error_variable}")

# After that, wait for ~3 seconds
execute_process(COMMAND sleep 3)

if(error_variable MATCHES "timeout" OR error_variable EQUAL 128 OR error_variable EQUAL 124)
    # Okay
elseif(NOT error_variable)
    # return code == 0, also okay
elseif(error_variable EQUAL 133 OR error_variable MATCHES "Child killed")
    # sigkill, return fatal error but mark the issue
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} had to be forcefully killed after 5 additional seconds after SIGINT")
else()
    # not timeout and error code != 0, not okay
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} produced an error (${error_variable}) while running")
endif() 

