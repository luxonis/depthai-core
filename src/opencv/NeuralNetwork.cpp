#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "capabilities/ImgFrameCapability.hpp"

namespace dai::node {

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const std::shared_ptr<ReplayVideo>& input, const Model& model, std::optional<float> fps) {
    decodeModel(model);

    ImgFrameCapability cap;
    if(fps.has_value()) cap.fps.value = *fps;
    cap = getFrameCapability(*nnArchive, cap);
    input->setOutFrameType(cap.type.value());
    if(fps.has_value()) input->setFps(*fps);
    input->setSize(std::get<std::pair<unsigned int, unsigned int>>(cap.size.value.value()));
    input->out.link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}

}  // namespace dai::node
