execute_process(COMMAND "${READELF}" -d "${BINARY}"
    RESULT_VARIABLE result OUTPUT_VARIABLE dynamic ERROR_VARIABLE error)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "readelf failed: ${error}")
endif()
if(dynamic MATCHES "NEEDED[^\n]*libdynamic_calibration[^\n]*\\.so")
    message(FATAL_ERROR "Unexpected shared Dynamic Calibration dependency: ${dynamic}")
endif()
message(STATUS "No shared Dynamic Calibration dependency")
