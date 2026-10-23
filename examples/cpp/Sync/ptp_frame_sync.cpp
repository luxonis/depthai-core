// Locks the device's FSYNC period to the PTP time grid with Pipeline::setPtpFrameSync and reports the lock state once a second,
// the frame sync tag counts per stream and each stream's offset from the grid on getTimestampSystem().
// Usage: ptp_frame_sync [phase_ns=0] [seconds=60] [fps=30]
// Exits 0 when the lock reached LOCKED and every stream carried PTP-tagged frames.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

namespace {

const char* toString(dai::PtpFrameSyncStatus::State state) {
    switch(state) {
        case dai::PtpFrameSyncStatus::State::IDLE:
            return "IDLE";
        case dai::PtpFrameSyncStatus::State::ACQUIRING:
            return "ACQUIRING";
        case dai::PtpFrameSyncStatus::State::LOCKED:
            return "LOCKED";
        case dai::PtpFrameSyncStatus::State::HOLDOVER:
            return "HOLDOVER";
        case dai::PtpFrameSyncStatus::State::FAULT:
            return "FAULT";
    }
    return "?";
}

int64_t gridOffsetNs(int64_t timestampNs, int64_t phaseNs, int64_t periodNs) {
    int64_t offset = (timestampNs - phaseNs) % periodNs;
    if(offset < 0) offset += periodNs;
    if(2 * offset > periodNs) offset -= periodNs;
    return offset;
}

struct StreamStats {
    std::shared_ptr<dai::MessageQueue> queue;
    uint64_t frames = 0;
    uint64_t ptpFrames = 0;
    uint64_t inputFrames = 0;
    int64_t lastGridOffsetNs = 0;
};

}  // namespace

int main(int argc, char** argv) {
    const int64_t phaseNs = argc > 1 ? std::stoll(argv[1]) : 0;
    const int seconds = argc > 2 ? std::stoi(argv[2]) : 60;
    const float fps = argc > 3 ? std::stof(argv[3]) : 30.0f;
    const int64_t periodNs = std::llround(1e9 / fps);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    pipeline.setPtpFrameSync(std::chrono::nanoseconds(phaseNs));

    std::map<std::string, StreamStats> streams;
    for(const auto& features : device->getConnectedCameraFeatures()) {
        const auto& types = features.supportedTypes;
        if(std::find(types.begin(), types.end(), dai::CameraSensorType::TOF) != types.end()) {
            auto tof = pipeline.create<dai::node::ToF>()->build(features.socket, dai::ToFConfig::Profile::MID_RANGE, fps);
            streams["TOF"].queue = tof->depth.createOutputQueue(4, false);
        } else {
            auto camera = pipeline.create<dai::node::Camera>()->build(features.socket, std::nullopt, fps);
            streams[dai::toString(features.socket)].queue =
                camera->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, fps)->createOutputQueue(4, false);
        }
    }

    pipeline.start();
    const auto start = std::chrono::steady_clock::now();
    auto nextReport = start;
    std::optional<double> lockedAfterS;
    while(pipeline.isRunning() && std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
        for(auto& [name, stats] : streams) {
            while(auto frame = stats.queue->tryGet<dai::ImgFrame>()) {
                ++stats.frames;
                if(frame->getFsync() == dai::ImgFrame::Fsync::PTP) ++stats.ptpFrames;
                if(frame->getFsync() == dai::ImgFrame::Fsync::INPUT) ++stats.inputFrames;
                if(const auto system = frame->getTimestampSystem()) {
                    const int64_t timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(system->time_since_epoch()).count();
                    stats.lastGridOffsetNs = gridOffsetNs(timestampNs, phaseNs, periodNs);
                }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if(now >= nextReport) {
            nextReport += std::chrono::seconds(1);
            const auto status = device->getPtpFrameSyncStatus();
            const double elapsedS = std::chrono::duration<double>(now - start).count();
            if(!lockedAfterS && status.state == dai::PtpFrameSyncStatus::State::LOCKED) lockedAfterS = elapsedS;
            std::cout << elapsedS << " s: " << toString(status.state) << " error " << status.lastErrorNs << " ns, period " << status.appliedCycles
                      << " cycles |";
            for(const auto& [name, stats] : streams) {
                std::cout << " " << name << " " << stats.frames << " frames (" << stats.ptpFrames << " PTP) grid offset " << stats.lastGridOffsetNs << " ns";
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    pipeline.stop();

    bool everyStreamPtp = !streams.empty();
    for(const auto& [name, stats] : streams) {
        std::cout << name << ": " << stats.frames << " frames, " << stats.ptpFrames << " PTP, " << stats.inputFrames << " INPUT" << std::endl;
        everyStreamPtp = everyStreamPtp && stats.ptpFrames > 0;
    }
    if(lockedAfterS) {
        std::cout << "first LOCKED status after " << *lockedAfterS << " s" << std::endl;
    } else {
        std::cout << "never LOCKED" << std::endl;
    }
    return lockedAfterS && everyStreamPtp ? EXIT_SUCCESS : EXIT_FAILURE;
}
