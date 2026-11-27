#include <algorithm>
#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"
int main(int argc, char** argv) {
    auto device = argc > 1 ? std::make_shared<dai::Device>(dai::DeviceInfo(argv[1])) : std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto flow = pipeline.create<dai::node::OpticalFlow>();
    auto deblur = pipeline.create<dai::node::Deblur>();
    auto config = deblur->getConfig();
    config.useOpenCL = true;    // false uses the on-device CPU reference FIR
    deblur->setConfig(config);  // exposureTimeUs=0 uses actual camera metadata
    camera->requestOutput({640, 384}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 15)->link(flow->inputImage);
    flow->passthroughInputImage.link(deblur->inputImage);
    flow->outputFlow.link(deblur->inputFlow);
    auto restored = deblur->output.createOutputQueue(2, true);
    auto original = deblur->passthroughInputImage.createOutputQueue(2, true);
    auto masks = deblur->outputMask.createOutputQueue(2, true);
    pipeline.start();
    for(int i = 0; i < 30; ++i) {
        const auto receive = [](const std::shared_ptr<dai::MessageQueue>& queue) {
            bool timedOut = false;
            auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
            if(timedOut || !frame) throw std::runtime_error("Timed out waiting for Deblur outputs");
            return frame;
        };
        const auto out = receive(restored), src = receive(original), mask = receive(masks);
        if(out->getSequenceNum() != src->getSequenceNum() || out->getSequenceNum() != mask->getSequenceNum()) throw std::runtime_error("Output pairing failed");
        const auto pixels = mask->getData();
        const auto accepted = std::count_if(pixels.begin(), pixels.end(), [](std::uint8_t value) { return value > 0; });
        std::cout << "Frame " << out->getSequenceNum() << ": accepted motion " << 100.0 * accepted / pixels.size() << "%, exposure "
                  << src->getExposureTime().count() * 0.001 << " ms\n";
    }
}
