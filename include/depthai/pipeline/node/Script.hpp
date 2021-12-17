#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/ScriptProperties.hpp>

namespace dai {
namespace node {

class Script : public NodeCRTP<Node, Script, ScriptProperties> {
   public:
    constexpr static const char* NAME = "Script";

   private:
    std::string scriptPath = "";

   public:
    Script(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    Script(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     *  Inputs to Script node. Can be accessed using subscript operator (Eg: inputs['in1'])
     *  By default inputs are set to blocking with queue size 8
     */
    InputMap inputs;

    /**
     * Outputs from Script node. Can be accessed subscript operator (Eg: outputs['out1'])
     */
    OutputMap outputs;

    /**
     *  Specify local filesystem path to load the script
     */
    void setScriptPath(const std::string& path);

    /**
     * Sets script data to be interpreted
     * @param script Script string to be interpreted
     * @param name Optionally set a name of this script
     */
    void setScript(const std::string& script, const std::string& name = "");

    /**
     * Sets script data to be interpreted
     * @param data Binary data that represents the script to be interpreted
     * @param name Optionally set a name of this script
     */
    void setScript(const std::vector<std::uint8_t>& data, const std::string& name = "");

    /**
     * Get filesystem path from where script was loaded.
     * If script wasn't set by path, function returns empty string
     */
    std::string getScriptPath() const;

    /**
     * Get filesystem path from where script was loaded.
     * If script wasn't set by path, function returns empty string
     */
    std::string getScriptName() const;

    /**
     * Set on which processor the script should run
     * @param type Processor type - Leon CSS or Leon MSS
     */
    void setProcessor(ProcessorType type);

    /**
     * Get on which processor the script should run
     * @returns Processor type - Leon CSS or Leon MSS
     */
    ProcessorType getProcessor() const;
};

}  // namespace node
}  // namespace dai
