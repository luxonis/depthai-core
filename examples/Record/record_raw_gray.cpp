#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::MonoCamera>();
    auto record = pipeline.create<dai::node::RecordVideo>();

    std::string path = argc > 1 ? argv[1] : "recording_raw_gray";
    record->setRecordVideoFile(path + std::string(".mp4"));
    record->setRecordMetadataFile(path + std::string(".mcap"));

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    cam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    cam->setFps(30);

    cam->out.link(record->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    pipeline.stop();
}
