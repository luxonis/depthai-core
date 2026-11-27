#pragma once

#include <hardware/camera3.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "CameraStatsAbi.hpp"
#include "depthai/common/CameraStats.hpp"

namespace dai {
namespace gate {

// Read the optional camera HAL extension without holding any callback-state lock.
CameraStats readCslCameraStats(int cameraId);
CameraStats readCslCameraStats(int cameraId, lux_sc_camera_stats_get_v1_cb_t getStats);

class CameraStatsTracker {
   public:
    static std::shared_ptr<CameraStatsTracker> create(CameraBoardSocket socket, int cameraId);
    CameraStatsTracker(CameraBoardSocket socket, int cameraId);
    void start(uint32_t partialResultCount);
    void submit(uint32_t frameNumber, uint32_t outputCount);
    void captureResult(const camera3_capture_result_t& result);
    void notify(const camera3_notify_msg_t& message);
    CameraStats snapshot(const CameraStats& source = {}, bool retainStoppedSource = false);
    void stop(const CameraStats& source = {});

    static constexpr size_t MAX_PENDING_REQUESTS = 1024;

   private:
    struct Request {
        uint32_t remainingBuffers = 0;
        bool metadataDone = false;
        bool failed = false;
    };

    // Caller must hold mutex.
    void updateSnapshotLocked(const CameraStats& source, bool retainStoppedSource);

    std::mutex mutex;
    CameraStats stats;
    std::string halSessionId;
    uint32_t partialResultCount = 1;
    std::map<uint32_t, Request> pending;
};

// Enumerates discovery and snapshots shared tracker state, never CameraSensor pointers.
std::vector<CameraStats> getCameraStats();

}  // namespace gate
}  // namespace dai
