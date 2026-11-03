// ToF with hardware FSYNC and frame-sync-id staggering (multiple OAK ToF devices).
//
// Run one instance per device. Give every device a different frame-sync-id, so their
// ToF illumination is staggered in time and the devices do not interfere.
// On OAK-D-SR-PoE the ToF sensor can only receive FSYNC, so the device wired as
// FSYNC master must also run a stereo sensor (CAM_B) to generate the pulse: pass --generator.
//
//   tof_frame_sync --device 10.0.0.10 --sync-id 0 --generator
//   tof_frame_sync --device 10.0.0.11 --sync-id 1

#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string deviceName;
    int syncId = 0;
    bool generator = false;
    float fps = 10.0f;
    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--device" && i + 1 < argc) {
            deviceName = argv[++i];
        } else if(arg == "--sync-id" && i + 1 < argc) {
            syncId = std::stoi(argv[++i]);
        } else if(arg == "--generator") {
            generator = true;
        } else if(arg == "--fps" && i + 1 < argc) {
            fps = std::stof(argv[++i]);
        }
    }

    auto device = deviceName.empty() ? std::make_shared<dai::Device>() : std::make_shared<dai::Device>(dai::DeviceInfo(deviceName));
    dai::Pipeline pipeline(device);

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, fps);
    tof->initialControl.setFrameSyncId(syncId);

    std::shared_ptr<dai::MessageQueue> generatorQueue;
    if(generator) {
        auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        // The frames are not needed, the sensor only has to run (a consumer is required)
        generatorQueue = cam->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps)->createOutputQueue(1, false);
    }

    auto depthQueue = tof->depth.createOutputQueue();
    pipeline.start();
    std::optional<double> prev;
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        double ts = std::chrono::duration<double>(depth->getTimestamp().time_since_epoch()).count();
        if(prev) {
            std::cout << "sync-id " << syncId << ": depth seq " << depth->getSequenceNum() << ", host ts " << ts << " s, interval " << (ts - *prev) * 1000.0
                      << " ms" << std::endl;
        }
        prev = ts;
    }
    return 0;
}
