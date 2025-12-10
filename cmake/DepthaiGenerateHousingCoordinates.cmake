cmake_minimum_required(VERSION 3.19)

function(DepthaiGenerateHousingCoordinates)

    ### VARIABLES
    set(OUTPUT_FILE "${ARGV0}")
    set(JSON_DIR "${ARGV1}")

    message(${OUTPUT_FILE})

    file(GLOB JSON_FILES "${JSON_DIR}/*.json")

    set(ALL_FILES "")
    
    # Loop through all housing coordinate files and parse their contents
    foreach(JSON_FILE ${JSON_FILES})
        get_filename_component(FILE_NAME ${JSON_FILE} NAME_WE)
        list(APPEND ALL_FILES "${FILE_NAME}")
        
        file(READ ${JSON_FILE} JSON_CONTENT)
        
        string(JSON POINTS_OBJ ERROR_VARIABLE JSON_ERROR GET "${JSON_CONTENT}" points)
        
        string(JSON POINTS_LENGTH ERROR_VARIABLE JSON_ERROR LENGTH "${JSON_CONTENT}" points)
        
        set(FILE_POINTS_${FILE_NAME} "")
        
        math(EXPR POINTS_LAST "${POINTS_LENGTH} - 1")
        foreach(IDX RANGE ${POINTS_LAST})
            # Parse the coordinate vector and format it as a map entry
            string(JSON POINT_ID ERROR_VARIABLE JSON_ERROR MEMBER "${JSON_CONTENT}" points ${IDX})
            
            string(JSON X_VAL ERROR_VARIABLE JSON_ERROR GET "${JSON_CONTENT}" points ${POINT_ID} x)
            string(JSON Y_VAL ERROR_VARIABLE JSON_ERROR GET "${JSON_CONTENT}" points ${POINT_ID} y)
            string(JSON Z_VAL ERROR_VARIABLE JSON_ERROR GET "${JSON_CONTENT}" points ${POINT_ID} z)
            
            set(FILE_POINTS_${FILE_NAME} "${FILE_POINTS_${FILE_NAME}}            {dai::HousingCoordinateSystem::${POINT_ID}, {${X_VAL}f, ${Y_VAL}f, ${Z_VAL}f}},\n")
        endforeach()
    endforeach()

    # Create the map body
    set(MAP_INIT "")
    foreach(FILE_NAME ${ALL_FILES})
        if(DEFINED FILE_POINTS_${FILE_NAME})
            set(MAP_INIT "${MAP_INIT}    {\n        \"${FILE_NAME}\",\n        {\n${FILE_POINTS_${FILE_NAME}}        }\n    },\n")
        endif()
    endforeach()

    # Write the final file contents
    file(WRITE ${OUTPUT_FILE}
"// depthai
#include \"depthai/common/DetectionParserOptions.hpp\"
#include \"depthai/common/HousingCoordinates.hpp\"

namespace dai {

static const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> dataMap = {
${MAP_INIT}};

const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> getHousingCoordinateSystems();

} // namespace dai
")
endfunction()