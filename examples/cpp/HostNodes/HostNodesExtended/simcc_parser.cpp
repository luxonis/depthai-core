#include "depthai/pipeline/node/host/contrib/host_nodes_ext/ParsingNeuralNetwork.hpp"
#include "messages/Keypoints.hpp"

int main() {

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto output = camera->requestOutput(std::make_pair(288, 384), dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::STRETCH);

    dai::NNModelDescription modelDescription{.model ="pedestl/rtmpose3d-open-mmlab-mmpose-large:v1-0-0:latest", .platform = pipeline.getDefaultDevice()->getPlatformAsString()};
    auto archive = dai::NNArchive(dai::getModelFromZoo(modelDescription));

    auto parsed_nn = pipeline.create<dai::node::ParsingNeuralNetwork>()->build(*output, archive);
    // Create output queue, note that parsed_nn->out only exists if the NN has only a single parser head
    auto out = parsed_nn->out.value().get().createOutputQueue();

    // Start pipeline
    pipeline.start();
    pipeline.processTasks();
    auto msg = out->get<dai::Buffer>();
    std::shared_ptr<dai::Keypoints3D3C> keypoints = std::dynamic_pointer_cast<dai::Keypoints3D3C>(msg);
    pipeline.stop();
    assert(keypoints != nullptr);
}
