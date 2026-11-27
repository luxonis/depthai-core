#include "CameraStats.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>

#include "camera/LuxonisSidechannel.hpp"
#include "utilities/CameraDiscovery.hpp"

namespace dai {
namespace gate {
namespace {
struct CameraStatsRegistry {
    std::mutex mutex;
    std::map<int, std::shared_ptr<CameraStatsTracker>> trackers;

    static CameraStatsRegistry& getInstance() {
        static CameraStatsRegistry instance;
        return instance;
    }
};
}  // namespace

CameraStats readCslCameraStats(int cameraId, lux_sc_camera_stats_get_v1_cb_t getStats) {
    CameraStats source;
    source.status = CameraStats::Status::UNSUPPORTED;
    if(getStats == nullptr) return source;
    source.status = CameraStats::Status::UNAVAILABLE;
    lux_sc_camera_stats_v1_t raw{};
    if(getStats(cameraId, &raw) != 0 || raw.abi_version != LUX_SC_CAMERA_STATS_ABI_VERSION || raw.counter_bits != 64 || raw.generation == 0) return source;
    source.status = raw.active ? CameraStats::Status::AVAILABLE : CameraStats::Status::INACTIVE;
    source.cslSessionId = std::to_string(raw.generation);
    source.cslTimestampNs = raw.timestamp_ns;
    source.cslErrorEvents = raw.error_events;
    source.linkStalledEvents = raw.error_causes[0];
    source.csidFatalEvents = raw.error_causes[1];
    source.csidFifoOverflowEvents = raw.error_causes[2];
    source.csidRecoveryOverflowEvents = raw.error_causes[3];
    source.csidLaneFifoOverflowEvents = raw.error_causes[4];
    source.csidPixelCountMismatchEvents = raw.error_causes[5];
    source.csidPacketHeaderCorruptionEvents = raw.error_causes[6];
    source.csidMissingPacketHeaderEvents = raw.error_causes[7];
    source.csidSensorSwitchingEvents = raw.error_causes[8];
    source.csidUnboundedFrameEvents = raw.error_causes[9];
    source.icpNoMemoryEvents = raw.error_causes[10];
    source.icpSystemFailureEvents = raw.error_causes[11];
    source.csidMissingEotEvents = raw.error_causes[12];
    source.csidPayloadCorruptionEvents = raw.error_causes[13];
    source.sensorStreamOffFailureEvents = raw.error_causes[14];
    return source;
}

CameraStats readCslCameraStats(int cameraId) {
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    return readCslCameraStats(cameraId, sidechannel == nullptr ? nullptr : sidechannel->ops.camera_stats_get_v1);
}

std::shared_ptr<CameraStatsTracker> CameraStatsTracker::create(CameraBoardSocket socket, int cameraId) {
    auto tracker = std::make_shared<CameraStatsTracker>(socket, cameraId);
    auto& registry = CameraStatsRegistry::getInstance();
    std::lock_guard<std::mutex> lock(registry.mutex);
    registry.trackers[cameraId] = tracker;
    return tracker;
}

CameraStatsTracker::CameraStatsTracker(CameraBoardSocket socket, int cameraId) {
    stats.socket = socket;
    stats.cameraId = cameraId;
    stats.status = CameraStats::Status::INACTIVE;
}

void CameraStatsTracker::start(uint32_t resultCount) {
    std::string sessionId;
    std::ifstream("/proc/sys/kernel/random/uuid") >> sessionId;
    std::lock_guard<std::mutex> lock(mutex);
    const auto socket = stats.socket;
    const auto cameraId = stats.cameraId;
    stats = {};
    stats.socket = socket;
    stats.cameraId = cameraId;
    halSessionId = std::move(sessionId);
    stats.sessionId = halSessionId;
    stats.status = halSessionId.empty() ? CameraStats::Status::UNAVAILABLE : CameraStats::Status::AVAILABLE;
    if(!halSessionId.empty()) {
        stats.completedCaptures = 0;
        stats.halDeviceErrors = 0;
        stats.halRequestErrors = 0;
        stats.halResultErrors = 0;
        stats.halBufferErrors = 0;
    }
    partialResultCount = std::max(uint32_t{1}, resultCount);
    pending.clear();
}

void CameraStatsTracker::submit(uint32_t frameNumber, uint32_t outputCount) {
    std::lock_guard<std::mutex> lock(mutex);
    if(stats.status != CameraStats::Status::AVAILABLE || !stats.completedCaptures) return;
    if(outputCount == 0 || pending.size() >= MAX_PENDING_REQUESTS || !pending.emplace(frameNumber, Request{outputCount}).second) {
        // Do not silently evict requests or fabricate a success count after lost correlation.
        stats.completedCaptures.reset();
        pending.clear();
    }
}

void CameraStatsTracker::captureResult(const camera3_capture_result_t& result) {
    std::lock_guard<std::mutex> lock(mutex);
    if(stats.status != CameraStats::Status::AVAILABLE) return;
    auto request = pending.find(result.frame_number);
    if(request == pending.end()) return;
    auto& state = request->second;
    if(result.result != nullptr && result.partial_result >= partialResultCount) state.metadataDone = true;
    if(result.num_output_buffers > state.remainingBuffers) {
        stats.completedCaptures.reset();
        pending.clear();
        return;
    }
    for(uint32_t i = 0; i < result.num_output_buffers; ++i) {
        if(result.output_buffers[i].status != CAMERA3_BUFFER_STATUS_OK) state.failed = true;
    }
    state.remainingBuffers -= result.num_output_buffers;
    if(state.remainingBuffers == 0 && state.metadataDone) {
        if(!state.failed && stats.completedCaptures) ++*stats.completedCaptures;
        pending.erase(request);
    }
}

void CameraStatsTracker::notify(const camera3_notify_msg_t& message) {
    if(message.type != CAMERA3_MSG_ERROR) return;
    std::lock_guard<std::mutex> lock(mutex);
    if(stats.status == CameraStats::Status::INACTIVE || halSessionId.empty()) return;
    const auto& error = message.message.error;
    switch(error.error_code) {
        case CAMERA3_MSG_ERROR_DEVICE:
            ++*stats.halDeviceErrors;
            stats.completedCaptures.reset();
            pending.clear();
            stats.status = CameraStats::Status::UNAVAILABLE;
            return;
        case CAMERA3_MSG_ERROR_REQUEST:
            ++*stats.halRequestErrors;
            break;
        case CAMERA3_MSG_ERROR_RESULT:
            ++*stats.halResultErrors;
            break;
        case CAMERA3_MSG_ERROR_BUFFER:
            ++*stats.halBufferErrors;
            break;
        default:
            return;
    }
    auto request = pending.find(error.frame_number);
    if(request == pending.end()) return;
    request->second.failed = true;
    if(error.error_code != CAMERA3_MSG_ERROR_BUFFER) request->second.metadataDone = true;
    if(request->second.remainingBuffers == 0 && request->second.metadataDone) pending.erase(request);
}

CameraStats CameraStatsTracker::snapshot(const CameraStats& source, bool retainStoppedSource) {
    std::lock_guard<std::mutex> lock(mutex);
    if(stats.status != CameraStats::Status::INACTIVE) updateSnapshotLocked(source, retainStoppedSource);
    return stats;
}

void CameraStatsTracker::updateSnapshotLocked(const CameraStats& source, bool retainStoppedSource) {
    const bool available = source.status == CameraStats::Status::AVAILABLE || (retainStoppedSource && source.status == CameraStats::Status::INACTIVE);
    stats.cslSessionId = available && !halSessionId.empty() ? halSessionId + "/" + source.cslSessionId : std::string{};
    stats.timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    stats.cslTimestampNs = available ? source.cslTimestampNs : std::nullopt;
    stats.cslErrorEvents = available ? source.cslErrorEvents : std::nullopt;
    stats.linkStalledEvents = available ? source.linkStalledEvents : std::nullopt;
    stats.csidFatalEvents = available ? source.csidFatalEvents : std::nullopt;
    stats.csidFifoOverflowEvents = available ? source.csidFifoOverflowEvents : std::nullopt;
    stats.csidRecoveryOverflowEvents = available ? source.csidRecoveryOverflowEvents : std::nullopt;
    stats.csidLaneFifoOverflowEvents = available ? source.csidLaneFifoOverflowEvents : std::nullopt;
    stats.csidPixelCountMismatchEvents = available ? source.csidPixelCountMismatchEvents : std::nullopt;
    stats.csidPacketHeaderCorruptionEvents = available ? source.csidPacketHeaderCorruptionEvents : std::nullopt;
    stats.csidMissingPacketHeaderEvents = available ? source.csidMissingPacketHeaderEvents : std::nullopt;
    stats.csidSensorSwitchingEvents = available ? source.csidSensorSwitchingEvents : std::nullopt;
    stats.csidUnboundedFrameEvents = available ? source.csidUnboundedFrameEvents : std::nullopt;
    stats.icpNoMemoryEvents = available ? source.icpNoMemoryEvents : std::nullopt;
    stats.icpSystemFailureEvents = available ? source.icpSystemFailureEvents : std::nullopt;
    stats.csidMissingEotEvents = available ? source.csidMissingEotEvents : std::nullopt;
    stats.csidPayloadCorruptionEvents = available ? source.csidPayloadCorruptionEvents : std::nullopt;
    stats.sensorStreamOffFailureEvents = available ? source.sensorStreamOffFailureEvents : std::nullopt;
}

void CameraStatsTracker::stop(const CameraStats& source) {
    std::lock_guard<std::mutex> lock(mutex);
    if(stats.status == CameraStats::Status::INACTIVE) return;
    updateSnapshotLocked(source, true);
    stats.status = CameraStats::Status::INACTIVE;
    pending.clear();
}

std::vector<CameraStats> getCameraStats() {
    std::vector<CameraStats> result;
    const auto cameras = CameraDiscovery::getInstance().getConnectedCameraDescriptions();
    result.reserve(cameras.size());
    for(const auto& camera : cameras) {
        const bool knownSocket = camera.physicalSocketKnown && std::count_if(cameras.begin(), cameras.end(), [&](const auto& other) {
                                                                   return other.physicalSocketKnown && other.camera.socket == camera.camera.socket;
                                                               }) == 1;
        CameraStats stats;
        stats.socket = knownSocket ? camera.camera.socket : CameraBoardSocket::AUTO;
        stats.cameraId = camera.id;
        if(!knownSocket) {
            result.push_back(std::move(stats));
            continue;
        }
        std::shared_ptr<CameraStatsTracker> tracker;
        {
            auto& registry = CameraStatsRegistry::getInstance();
            std::lock_guard<std::mutex> lock(registry.mutex);
            auto it = registry.trackers.find(camera.id);
            if(it != registry.trackers.end()) tracker = it->second;
        }
        stats.status = CameraStats::Status::INACTIVE;
        if(tracker) {
            const auto before = readCslCameraStats(camera.id);
            stats = tracker->snapshot(before);
            const auto after = readCslCameraStats(camera.id);
            if(stats.status == CameraStats::Status::AVAILABLE && (before.status != after.status || before.cslSessionId != after.cslSessionId)) {
                // A CSL source reset raced the read. Keep the independent HAL counters.
                stats = tracker->snapshot();
            }
        }
        result.push_back(std::move(stats));
    }
    std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
        return left.socket != right.socket ? left.socket < right.socket : left.cameraId < right.cameraId;
    });
    return result;
}

}  // namespace gate
}  // namespace dai
