#include <depthai/depthai.hpp>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/host/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

std::string getDefaultRecordingPath() {
    auto isTest = std::getenv("RUNNING_AS_TEST");
    if(isTest && std::string(isTest) == "1") {
        // If running as test save to temporary directory
        char tmpTemplate[] = "raw_recording_XXXXXX";
        char* tmpName = mkdtemp(tmpTemplate);
        auto tmpDir = std::filesystem::temp_directory_path();
        return (tmpDir / tmpName).string();
    } else {
        return "raw_recording";
    }
}

int main(int argc, char** argv) {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto display = pipeline.create<dai::node::Display>();
    auto record = pipeline.create<dai::node::RecordVideo>();

    std::string path = argc > 1 ? argv[1] : getDefaultRecordingPath();
    record->setRecordVideoFile(path + std::string(".mp4"));
    record->setRecordMetadataFile(path + std::string(".mcap"));

    auto* camOut = cam->requestOutput({1280, 960}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);

    camOut->link(display->input);
    camOut->link(record->input);

    pipeline.run();  // Let the display node stop the pipeline
}
