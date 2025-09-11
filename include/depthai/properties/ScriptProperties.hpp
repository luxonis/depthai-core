#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * Specify ScriptProperties options such as script uri, script name, ...
 */
struct ScriptProperties : PropertiesSerializable<Properties, ScriptProperties> {
    /**
     * Uri which points to actual script
     */
    std::string scriptUri;

    /**
     * Name of script
     */
    std::string scriptName = "<script>";

    /**
     * Which processor should execute the script
     */
    ProcessorType processor = ProcessorType::LEON_MSS;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(ScriptProperties, scriptUri, scriptName, processor);

}  // namespace dai
