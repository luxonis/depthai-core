#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    auto device = argc > 1 ? std::make_shared<dai::Device>(dai::DeviceInfo(argv[1])) : std::make_shared<dai::Device>();
    if(device->getPlatform() != dai::Platform::RVC4) throw std::runtime_error("Optical flow output requires RVC4");
    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto flowNode = pipeline.create<dai::node::OpticalFlow>();
    camera->requestOutput({640, 384}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30)->link(flowNode->inputImage);
    auto queue = flowNode->outputFlow.createOutputQueue(2, true);
    pipeline.start();
    for(int i = 0; i < 30; ++i) {
        auto group = queue->get<dai::MessageGroup>();
        auto reference = group->get<dai::Buffer>("reference");
        auto target = group->get<dai::Buffer>("target");
        auto flow = group->get<dai::NNData>("flow");
        const auto dt = std::chrono::duration<double>(target->getTimestampDevice() - reference->getTimestampDevice()).count();
        std::cout << reference->getSequenceNum() << " -> " << target->getSequenceNum() << ", interval " << dt << " s, " << flow->getAllLayerNames().size()
                  << " tensors\n";
    }
}
