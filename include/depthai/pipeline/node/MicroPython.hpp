#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/MicroPythonProperties.hpp>

namespace dai {
namespace node {

class MicroPython : public Node {
    dai::MicroPythonProperties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;
    // void loadAssets(AssetManager& assetManager) override;

    std::string scriptPath = "";

   public:
    MicroPython(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     *  Inputs to MicroPython node. Can be added or removed
     *  By default inputs are set to blocking with queue size 8
     */
    InputMap inputs;

    /**
     * Outputs from MicroPython node. Can be added or removed
     */
    OutputMap outputs;

    /**
     *  Set the node name. Used in micropython to access this nodes IO and assets
     */
    void setName(const std::string& name);

    /**
     *  Specify local filesystem path to load the script
     */
    void setScriptPath(const std::string& path);

    /**
     *  Get filesystem path from where script was loaded
     */
    std::string getScriptPath() const;

    /**
     * Add binary assets that will be accessable on the MX
     */
    void addAsset(const std::string& name, const std::string& path);

    /**
     * Set which processor should script run on Leon CSS or Leon MSS
     */
    void setProcessor(ProcessorType type);

    /**
     * Get which processor should script run on Leon CSS or Leon MSS
     */
    ProcessorType getProcessor() const;
};

}  // namespace node
}  // namespace dai
