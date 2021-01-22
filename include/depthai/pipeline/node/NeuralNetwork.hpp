#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/pb/properties/NeuralNetworkProperties.hpp>

namespace dai {
namespace node {

class NeuralNetwork : public Node {
    dai::NeuralNetworkProperties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() override;
    // void loadAssets(AssetManager& assetManager) override;

   protected:
    struct BlobAssetInfo {
        std::string uri;
        uint32_t size;
    };
    std::string blobPath;
    BlobAssetInfo loadBlob(const std::string& path);
    OpenVINO::Version networkOpenvinoVersion;

   public:
    NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNData, false}}};
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    void setBlobPath(const std::string& path);
    void setNumPoolFrames(int numFrames);
    void setNumInferenceThreads(int numThreads);
};

}  // namespace node
}  // namespace dai
