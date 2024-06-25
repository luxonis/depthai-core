#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/host/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main() {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<dai::node::Display>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto record = pipeline.create<dai::node::Record>();

    record->setRecordFile("recording");

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setFps(30);

    videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

    cam->video.link(videoEncoder->input);
    cam->video.link(display->input);
    videoEncoder->out.link(record->input);

    pipeline.run(); // Let the display node stop the pipeline
}
