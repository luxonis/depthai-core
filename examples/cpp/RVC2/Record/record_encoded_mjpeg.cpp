#include <cassert>
#include <depthai/depthai.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

#include "depthai/pipeline/node/host/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    std::string path = argc > 1 ? argv[1] : "recording_mjpeg";
    {
        dai::Pipeline pipeline(true);
        auto cam = pipeline.create<dai::node::ColorCamera>();
        auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
        auto record = pipeline.create<dai::node::RecordVideo>();

        record->setRecordVideoFile(path + std::string(".mp4"));
        record->setRecordMetadataFile(path + std::string(".mcap"));

        cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        cam->setFps(30);

        videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);

        cam->video.link(videoEncoder->input);
        videoEncoder->out.link(record->input);

        pipeline.start();

        pipeline.wait();
    }
    return 0;
}
