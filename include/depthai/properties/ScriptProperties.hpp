#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

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

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(ScriptProperties, scriptUri, scriptName, processor);

}  // namespace dai
