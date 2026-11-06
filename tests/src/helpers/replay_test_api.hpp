#pragma once

// Thin wrappers for the replay-node API. Spec names are proposals; tests call these.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/utility/ReplayCache.hpp"

namespace replay_test {

using Location = dai::ReplaySourceLocation;
using DecodeMode = dai::ReplayDecodeMode;
using CacheKind = dai::ReplayCacheKind;

inline dai::node::ReplayVideo& setSource(dai::node::ReplayVideo& replay, const std::filesystem::path& path, Location location = Location::HOST) {
    return replay.setSource(path, location);
}
inline dai::node::ReplayVideo& setMetadataFile(dai::node::ReplayVideo& replay, const std::filesystem::path& path) {
    return replay.setMetadataFile(path);
}
inline dai::node::ReplayVideo& setOutFrameType(dai::node::ReplayVideo& replay, dai::ImgFrame::Type type) {
    return replay.setOutFrameType(type);
}
inline dai::node::ReplayVideo& setSize(dai::node::ReplayVideo& replay, int width, int height) {
    return replay.setSize(width, height);
}
inline dai::node::ReplayVideo& setFps(dai::node::ReplayVideo& replay, float fps) {
    return replay.setFps(fps);
}
inline dai::node::ReplayVideo& setLoop(dai::node::ReplayVideo& replay, bool loop) {
    return replay.setLoop(loop);
}
inline dai::node::ReplayVideo& setStopPipelineAtEnd(dai::node::ReplayVideo& replay, bool stop) {
    return replay.setStopPipelineAtEnd(stop);
}
inline dai::node::ReplayVideo& setDecodeMode(dai::node::ReplayVideo& replay, DecodeMode mode) {
    return replay.setDecodeMode(mode);
}
inline dai::node::ReplayVideo& setRunOnHost(dai::node::ReplayVideo& replay, bool runOnHost) {
    return replay.setRunOnHost(runOnHost);
}
inline dai::node::ReplayVideo& setPersistentUpload(dai::node::ReplayVideo& replay, bool persistent) {
    return replay.setPersistentUpload(persistent);
}
inline dai::node::ReplayVideo& setArchiveEntry(dai::node::ReplayVideo& replay, const std::string& entry) {
    return replay.setArchiveEntry(entry);
}
inline dai::node::ReplayVideo& setReplayVideoFile(dai::node::ReplayVideo& replay, const std::filesystem::path& path) {
    return replay.setReplayVideoFile(path);
}
inline dai::node::ReplayVideo& setReplayMetadataFile(dai::node::ReplayVideo& replay, const std::filesystem::path& path) {
    return replay.setReplayMetadataFile(path);
}

inline void setFast(dai::node::ReplayVideo& replay) {
    setLoop(replay, false);
    setStopPipelineAtEnd(replay, false);
    setFps(replay, 1000.f);
}

inline dai::node::ReplayMetadataOnly& setReplayFile(dai::node::ReplayMetadataOnly& node, const std::filesystem::path& path) {
    return node.setReplayFile(path);
}
inline dai::node::ReplayMetadataOnly& setSource(dai::node::ReplayMetadataOnly& node, const std::filesystem::path& path, Location location = Location::HOST) {
    return node.setSource(path, location);
}
inline dai::node::ReplayMetadataOnly& setArchiveEntry(dai::node::ReplayMetadataOnly& node, const std::string& entry) {
    return node.setArchiveEntry(entry);
}
inline dai::node::ReplayMetadataOnly& setPersistentUpload(dai::node::ReplayMetadataOnly& node, bool persistent) {
    return node.setPersistentUpload(persistent);
}
inline dai::node::ReplayMetadataOnly& setRunOnHost(dai::node::ReplayMetadataOnly& node, bool runOnHost) {
    return node.setRunOnHost(runOnHost);
}
inline dai::node::ReplayMetadataOnly& setFps(dai::node::ReplayMetadataOnly& node, float fps) {
    return node.setFps(fps);
}
inline dai::node::ReplayMetadataOnly& setLoop(dai::node::ReplayMetadataOnly& node, bool loop) {
    return node.setLoop(loop);
}
inline dai::node::ReplayMetadataOnly& setStopPipelineAtEnd(dai::node::ReplayMetadataOnly& node, bool stop) {
    return node.setStopPipelineAtEnd(stop);
}

inline void setHostCacheLimit(std::uint64_t bytes) {
    dai::node::ReplayVideo::setReplayCacheLimit(bytes);
}
inline dai::ReplayCacheInfo getHostCacheInfo() {
    return dai::node::ReplayVideo::getReplayCacheInfo();
}
inline void clearHostCache() {
    dai::node::ReplayVideo::clearReplayCache();
}

inline void setReplayCacheLimit(dai::Device& device, std::uint64_t bytes) {
    device.setReplayCacheLimit(bytes);
}
inline dai::ReplayCacheInfo getReplayCacheInfo(dai::Device& device) {
    return device.getReplayCacheInfo();
}
inline void clearReplayCache(dai::Device& device) {
    device.clearReplayCache();
}

inline std::shared_ptr<dai::node::Camera> buildCamera(dai::Pipeline& pipeline, dai::CameraBoardSocket socket, dai::node::ReplayVideo& replay) {
    auto camera = pipeline.create<dai::node::Camera>();
    camera->build(socket, replay);
    return camera;
}
inline void setMockIsp(dai::node::Camera& camera, dai::node::ReplayVideo& replay) {
    camera.setMockIsp(replay);
}

}  // namespace replay_test
