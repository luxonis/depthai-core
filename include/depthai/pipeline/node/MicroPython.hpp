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

   public:
    MicroPython(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
    //    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNData, false}}};
    //    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    // Specify local filesystem path to load the script
    void setScriptPath(const std::string& path);
};

}  // namespace node
}  // namespace dai
