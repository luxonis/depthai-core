#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai::node {

std::shared_ptr<DetectionNetwork> DetectionNetwork::build(const std::shared_ptr<ReplayVideo>& input, const Model& model, std::optional<float> fps) {
    neuralNetwork->build(input, model, fps);
    auto nnArchive = neuralNetwork->getNNArchive();
    DAI_CHECK(nnArchive.has_value(), "NeuralNetwork NNArchive is not set after build.");
    detectionParser->setNNArchive(*nnArchive);
    return std::static_pointer_cast<DetectionNetwork>(shared_from_this());
}

}  // namespace dai::node
