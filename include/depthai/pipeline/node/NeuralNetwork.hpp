#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/NeuralNetworkProperties.hpp>

namespace dai {
namespace node {

class NeuralNetwork : public Node {
    dai::NeuralNetworkProperties properties;
    virtual dai::NeuralNetworkProperties& getPropertiesRef();

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() override;
    // void loadAssets(AssetManager& assetManager) override;

   protected:
    OpenVINO::Version networkOpenvinoVersion;

   public:
    NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Input message with data to be infered upon
     * Default queue is blocking with size 5
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 5, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNData, false}}};
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    void setBlobPath(const std::string& path);
    void setNumPoolFrames(int numFrames);
    void setNumInferenceThreads(int numThreads);
    void setNumNCEPerInferenceThread(int numNCEPerThread);
    // Zero means AUTO. TODO add AUTO in NeuralNetworkProperties
    int getNumInferenceThreads();
    // TODO add getters for other API
};

}  // namespace node
}  // namespace dai
