#pragma once

// Shared helpers for the record examples in this folder.
// The programs build a Pipeline and write under RR_OUTPUT (default rr-output).
// Older firmware without the RecordVideo node is reported as a known limit.
// Other errors are reported separately.

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

namespace rr {

inline std::filesystem::path outputDir() {
    if(const char* env = std::getenv("RR_OUTPUT")) return env;
    const std::filesystem::path path{"rr-output"};
    std::filesystem::create_directories(path);
    return path;
}

inline std::string uniqueName(const std::string& prefix) {
    return prefix + "_" + dai::makeRecordingTimestamp();
}

inline const char* statusName(dai::RecordStatus::State state) {
    switch(state) {
        case dai::RecordStatus::State::STARTED:
            return "STARTED";
        case dai::RecordStatus::State::STOPPED:
            return "STOPPED";
        case dai::RecordStatus::State::ERROR:
            return "ERROR";
    }
    return "UNKNOWN";
}

inline void printStatus(const dai::RecordStatus& status) {
    std::cout << "status " << statusName(status.state) << " segment=" << status.segment << " frames=" << status.frameCount << " seq=" << status.firstSequenceNum
              << ".." << status.lastSequenceNum << " skipped=" << status.skippedFrames << " path=" << status.path;
    if(!status.error.empty()) std::cout << " error=" << status.error;
    std::cout << "\n";
}

inline int classifyRecordFailure(const std::string& text) {
    std::cout << "EXCEPTION: " << text << "\n";
    const bool firmware = text.find("implemented by device firmware") != std::string::npos;
    // The device reports: Pipeline node with name: 'RecordVideo' doesn't exist.
    const bool missingNode =
        text.find("RecordVideo") != std::string::npos
        && (text.find("doesn't exist") != std::string::npos || text.find("does not exist") != std::string::npos || text.find("not found") != std::string::npos
            || text.find("unknown") != std::string::npos || text.find("Unknown") != std::string::npos || text.find("no node") != std::string::npos
            || text.find("not supported") != std::string::npos);
    if(firmware || missingNode) {
        std::cout << "KNOWN_LIMIT: RecordVideo device execution is not in this firmware. The example still configured that path.\n";
        return 2;
    }
    return 1;
}

// Names of VideoEncoderProperties::Profile, in enum order. "none" is not a profile.
inline const char* videoEncoderProfileList() {
    return "['H264_BASELINE', 'H264_HIGH', 'H264_MAIN', 'H265_MAIN', 'MJPEG']";
}

inline std::optional<dai::VideoEncoderProperties::Profile> videoEncoderProfile(const std::string& name) {
    using Profile = dai::VideoEncoderProperties::Profile;
    if(name == "H264_BASELINE") return Profile::H264_BASELINE;
    if(name == "H264_HIGH") return Profile::H264_HIGH;
    if(name == "H264_MAIN") return Profile::H264_MAIN;
    if(name == "H265_MAIN") return Profile::H265_MAIN;
    if(name == "MJPEG") return Profile::MJPEG;
    return std::nullopt;
}

// Names of RecordConfig::CompressionLevel, in enum order.
inline const char* compressionLevelList() {
    return "['NONE', 'FASTEST', 'FAST', 'DEFAULT', 'SLOW', 'SLOWEST']";
}

inline std::optional<dai::RecordConfig::CompressionLevel> compressionLevel(const std::string& name) {
    using Level = dai::RecordConfig::CompressionLevel;
    if(name == "NONE") return Level::NONE;
    if(name == "FASTEST") return Level::FASTEST;
    if(name == "FAST") return Level::FAST;
    if(name == "DEFAULT") return Level::DEFAULT;
    if(name == "SLOW") return Level::SLOW;
    if(name == "SLOWEST") return Level::SLOWEST;
    return std::nullopt;
}

inline dai::Node::Output* addCameraNv12(dai::Pipeline& pipeline, int fps = 10) {
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    // 640x480 is a multiple of 32 by 8, which RVC2 encoding requires.
    return camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, static_cast<float>(fps));
}

inline void linkEncoder(dai::Pipeline& pipeline,
                        dai::Node::Output* frames,
                        const std::shared_ptr<dai::node::RecordVideo>& record,
                        dai::VideoEncoderProperties::Profile codec = dai::VideoEncoderProperties::Profile::H264_MAIN) {
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    // RecordVideo rejects H.265 in VIDEO format.
    encoder->setProfile(codec);
    frames->link(encoder->input);
    encoder->out.link(record->input);
}

inline int startOrClassify(dai::Pipeline& pipeline) {
    try {
        pipeline.start();
        return 0;
    } catch(const std::exception& ex) {
        return classifyRecordFailure(ex.what());
    }
}

inline int recordFor(dai::Pipeline& pipeline, const std::shared_ptr<dai::node::RecordVideo>& record, std::chrono::milliseconds duration) {
    std::cout << "RecordVideo runOnHost=" << (record->runOnHost() ? "true" : "false") << "\n";
    auto status = record->status.createOutputQueue(16, false);
    try {
        pipeline.start();
    } catch(const std::exception& ex) {
        return classifyRecordFailure(ex.what());
    }
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while(std::chrono::steady_clock::now() < deadline && pipeline.isRunning()) {
        try {
            if(auto message = status->tryGet<dai::RecordStatus>()) printStatus(*message);
        } catch(const std::exception& ex) {
            std::cout << "status poll: " << ex.what() << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    try {
        pipeline.stop();
    } catch(const std::exception& ex) {
        std::cout << "STOP_EXCEPTION: " << ex.what() << "\n";
    }
    if(auto last = record->getStatus()) {
        std::cout << "getStatus ";
        printStatus(*last);
    } else {
        std::cout << "getStatus: empty\n";
    }
    // stop() closes the queue. A later tryGet throws "MessageQueue was closed".
    try {
        while(auto message = status->tryGet<dai::RecordStatus>()) printStatus(*message);
    } catch(const std::exception& ex) {
        std::cout << "status drain: " << ex.what() << "\n";
    }
    return 0;
}

// Drain live queues before stop closes them; the final snapshot remains available
// through getStatus(). Require an actual completed recording for these examples.
inline void finishRecording(dai::Pipeline& pipeline, const std::shared_ptr<dai::node::RecordVideo>& record, const std::shared_ptr<dai::MessageQueue>& status) {
    bool completed = false;
    bool failed = false;
    const auto observe = [&](const dai::RecordStatus& message, bool show) {
        if(show) printStatus(message);
        if(message.state == dai::RecordStatus::State::STOPPED && message.frameCount > 0) completed = true;
        if(message.state == dai::RecordStatus::State::ERROR) failed = true;
    };
    while(auto message = status->tryGet<dai::RecordStatus>()) observe(*message, true);
    pipeline.stop();
    if(auto last = record->getStatus()) observe(*last, false);
    if(failed) throw std::runtime_error("Recorder published ERROR");
    if(!completed) throw std::runtime_error("No completed segment with recorded frames");
}

// Calls pipeline.stop() when destroyed, including when an exception leaves the scope.
class StopPipeline {
   public:
    explicit StopPipeline(dai::Pipeline& pipeline) : pipeline(&pipeline) {}
    ~StopPipeline() {
        if(pipeline == nullptr) return;
        try {
            pipeline->stop();
        } catch(const std::exception&) {
            // A second stop() during teardown must not replace the example's exit code.
        }
        pipeline = nullptr;
    }
    StopPipeline(const StopPipeline&) = delete;
    StopPipeline& operator=(const StopPipeline&) = delete;
    StopPipeline(StopPipeline&&) = delete;
    StopPipeline& operator=(StopPipeline&&) = delete;

   private:
    dai::Pipeline* pipeline;
};

}  // namespace rr
