#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main() {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::MonoCamera>();
    auto record = pipeline.create<dai::node::Record>();
    auto display = pipeline.create<dai::node::Display>();

    record->setRecordFile("recording_raw_gray");

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    cam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    cam->setFps(30);

    cam->out.link(record->input);
    cam->out.link(display->input);
    pipeline.run();
}
