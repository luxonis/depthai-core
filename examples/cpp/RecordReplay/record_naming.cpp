// Showcases multiple record nodes with different names.

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "depthai/depthai.hpp"
#include "example_util.hpp"

int main() {
    try {
        dai::Pipeline pipeline;
        rr::StopPipeline stop{pipeline};

        const auto stem = rr::outputDir();
        const auto name = rr::uniqueName("naming");
        std::vector<std::shared_ptr<dai::node::RecordVideo>> recorders;
        std::shared_ptr<dai::node::RecordVideo> record;
        for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B}) {
            auto camera = pipeline.create<dai::node::Camera>()->build(socket);
            auto* stream = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);

            auto encoder = pipeline.create<dai::node::VideoEncoder>();
            encoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
            stream->link(encoder->input);

            record = pipeline.create<dai::node::RecordVideo>();
            record->setKeepOnDevice(false);
            record->setFps(10);
            record->setOutputDirectory(stem);
            record->setStreamName(dai::toString(socket));
            record->setRecordingName(name);
            encoder->out.link(record->input);

            recorders.push_back(record);
        }

        std::cout << "Saving video to " << recorders.front()->getOutputDirectory().string() << "\n";
        return rr::recordFor(pipeline, record, std::chrono::seconds(5));
    } catch(const std::exception& ex) {
        return rr::classifyRecordFailure(ex.what());
    }
}
