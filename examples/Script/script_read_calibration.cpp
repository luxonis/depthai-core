#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
        cal = Device.readCalibration2()
        left_camera_id = cal.getStereoLeftCameraId()
        right_camera_id = cal.getStereoRightCameraId()

        extrinsics = cal.getCameraExtrinsics(left_camera_id, right_camera_id)
        intrinsics_left = cal.getCameraIntrinsics(left_camera_id)

        print(extrinsics)
        print(intrinsics_left)
    )");

    // Connect to device with pipeline
    dai::Device device(pipeline);
    while(true) {}

    return 0;
}
