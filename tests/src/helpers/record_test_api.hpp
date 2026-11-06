#pragma once

// Thin wrappers for the record-node API. Spec names are proposals; call these, not the node, from tests.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/RecordControl.hpp"
#include "depthai/pipeline/datatype/RecordStatus.hpp"
#include "depthai/utility/RecordReplay.hpp"

namespace record_test {

using Format = dai::RecordFormat;
using Compression = dai::RecordConfig::CompressionLevel;

inline dai::node::RecordVideo& setFormat(dai::node::RecordVideo& rec, Format format) {
    return rec.setFormat(format);
}
inline dai::node::RecordVideo& setRecordingName(dai::node::RecordVideo& rec, const std::string& name) {
    return rec.setRecordingName(name);
}
inline dai::node::RecordVideo& setOutputDirectory(dai::node::RecordVideo& rec, const std::filesystem::path& path) {
    return rec.setOutputDirectory(path);
}
inline dai::node::RecordVideo& setStreamName(dai::node::RecordVideo& rec, const std::string& name) {
    return rec.setStreamName(name);
}
// Spec setFps(float), including 0, negative, and NaN. The landed whole-number setter cannot express those.
inline dai::node::RecordVideo& setFps(dai::node::RecordVideo& rec, float fps) {
    return rec.setStreamFps(fps);
}
inline dai::node::RecordVideo& setFps(dai::node::RecordVideo& rec, unsigned int fps) {
    return rec.setFps(fps);
}
inline dai::node::RecordVideo& setCompressionLevel(dai::node::RecordVideo& rec, Compression level) {
    return rec.setCompressionLevel(level);
}
inline dai::node::RecordVideo& setWaitForTrigger(dai::node::RecordVideo& rec, bool wait) {
    return rec.setWaitForTrigger(wait);
}
inline dai::node::RecordVideo& setPreTriggerFrames(dai::node::RecordVideo& rec, std::int32_t frames) {
    return rec.setPreTriggerFrames(frames);
}
inline dai::node::RecordVideo& setMaxPreTriggerBytes(dai::node::RecordVideo& rec, std::uint64_t bytes) {
    return rec.setMaxPreTriggerBytes(bytes);
}
inline dai::node::RecordVideo& setTransferToHost(dai::node::RecordVideo& rec, bool transfer) {
    return rec.setTransferToHost(transfer);
}
inline dai::node::RecordVideo& setKeepOnDevice(dai::node::RecordVideo& rec, bool keep) {
    return rec.setKeepOnDevice(keep);
}
inline dai::node::RecordVideo& setRunOnHost(dai::node::RecordVideo& rec, bool runOnHost) {
    return rec.setRunOnHost(runOnHost);
}
inline dai::node::RecordVideo& setRecordVideoFile(dai::node::RecordVideo& rec, const std::filesystem::path& path) {
    return rec.setRecordVideoFile(path);
}
inline dai::node::RecordVideo& setRecordMetadataFile(dai::node::RecordVideo& rec, const std::filesystem::path& path) {
    return rec.setRecordMetadataFile(path);
}

inline std::shared_ptr<dai::RecordStatus> getStatus(const dai::node::RecordVideo& rec) {
    const auto status = rec.getStatus();
    if(!status) return nullptr;
    return std::make_shared<dai::RecordStatus>(*status);
}

inline std::shared_ptr<dai::RecordControl> startControl() {
    return dai::RecordControl::start();
}
// Spec start(int64, int64). The message field is int32; every catalog value fits.
inline std::shared_ptr<dai::RecordControl> startControl(std::int64_t maxFrames, std::int64_t maxDurationMs) {
    return dai::RecordControl::start(static_cast<std::int32_t>(maxFrames), static_cast<std::int32_t>(maxDurationMs));
}
inline std::shared_ptr<dai::RecordControl> stopControl() {
    return dai::RecordControl::stop();
}

// Spec field creationTimeMs. The store struct names it createdMs.
struct RecordingView {
    std::string name;
    std::uint64_t sizeBytes = 0;
    std::int64_t creationTimeMs = 0;
    bool complete = false;
};

inline std::vector<RecordingView> listRecordings(dai::Device& device) {
    std::vector<RecordingView> out;
    for(const auto& info : device.listRecordings()) {
        out.push_back(RecordingView{info.name, info.sizeBytes, info.createdMs, info.complete});
    }
    return out;
}
inline void downloadRecording(dai::Device& device, const std::string& name, const std::filesystem::path& hostDir) {
    device.downloadRecording(name, hostDir);
}
inline void deleteRecording(dai::Device& device, const std::string& name) {
    device.deleteRecording(name);
}

inline void setRecordConfigFormat(dai::RecordConfig& config, Format format) {
    config.format = format;
}
inline void setRecordConfigKeepOnDevice(dai::RecordConfig& config, bool keep) {
    config.keepOnDevice = keep;
}

// DeviceInfo.platform is XLinkPlatform_t. The spec comparison with dai::Platform does not compile.
inline bool infoIsPlatform(const dai::DeviceInfo& info, dai::Platform platform) {
    if(platform == dai::Platform::RVC4) return info.platform == X_LINK_RVC4;
    if(platform == dai::Platform::RVC2) return info.platform == X_LINK_MYRIAD_X || info.platform == X_LINK_MYRIAD_2;
    if(platform == dai::Platform::RVC3) return info.platform == X_LINK_RVC3;
    return false;
}

inline std::shared_ptr<dai::Device> openFirst(dai::Platform platform) {
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(infoIsPlatform(info, platform)) return std::make_shared<dai::Device>(info);
    }
    return nullptr;
}

}  // namespace record_test
