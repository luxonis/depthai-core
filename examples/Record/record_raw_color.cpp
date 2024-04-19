#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/host/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main() {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto record = pipeline.create<dai::node::Record>();

    record->setRecordFile("/home/work/workspaces/lib/depthai-python/depthai-core/recording_raw_color");

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setFps(30);

    cam->video.link(record->in);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    pipeline.stop();
}
