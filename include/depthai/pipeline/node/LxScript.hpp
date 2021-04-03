#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/LxScriptProperties.hpp>

namespace dai {
namespace node {

class LxScript : public Node {
    dai::LxScriptProperties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;
    // void loadAssets(AssetManager& assetManager) override;

    std::string scriptPath = "";

   public:
    LxScript(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     *  Inputs to LxScript node. Can be added or removed
     *  By default inputs are set to blocking with queue size 8
     */
    InputMap inputs;

    /**
     * Outputs from LxScript node. Can be added or removed
     */
    OutputMap outputs;

    /**
     *  Set the node name. Used in LxScript to access this nodes IO and assets
     */
    void setName(const std::string& name);

    /**
     *  Specify local filesystem path to load the script
     */
    void setScriptPath(const std::string& path);

    /**
     * Sets script data to be interpreted
     * @param script Script string to be interpreted
     */
    void setScriptData(const std::string& script);

    /**
     * Sets script data to be interpreted
     * @param data Binary data that represents the script to be interpreted
     */
    void setScriptData(const std::vector<std::uint8_t>& data);

    /**
     * Get filesystem path from where script was loaded.
     * If script wasn't set by path, function returns empty string
     */
    std::string getScriptPath() const;

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
