#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <tuple>
#include <cstdint>
#include <set>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"

enum class SyncType {
    EXTERNAL,
    PTP,
};

std::string toString(SyncType syncType);

struct FsyncTestParameters {
    double syncThresholdSec;
    // Measurement window after convergence; excludes startup, warmup, and convergence.
    uint64_t measurementDurationSec;
    uint64_t firstGroupTimeoutSec;
    uint64_t syncAcquisitionTimeoutSec;
    uint64_t warmupDurationSec;
    double deltaMeanThreshold;
    double deltaP99Threshold;
    SyncType syncType;
    std::optional<std::set<std::string>> allowedSensors;
    int expectedDevices;
    // Maximum measurement delivery silence, allowing host/network jitter independently of frame timestamp gaps.
    uint64_t measurementNoProgressTimeoutSec = 1;
};

struct GroupReadResult {
    std::chrono::system_clock::duration timestampSpread;
    std::string minStreamName;
    std::string maxStreamName;
    std::chrono::system_clock::time_point medianTimestamp;
};

// Reads and validates groups for the sync tests. The queue must outlive the reader.
class GroupReader {
   public:
    GroupReader(dai::MessageQueue& queue, std::vector<std::string> inputNames, SyncType syncType);

    // Consume at most one message before the deadline. An expired deadline consumes nothing.
    // Returns nullopt on timeout; malformed messages fail Catch2 assertions, and queue closure propagates.
    std::optional<GroupReadResult> read(std::chrono::steady_clock::time_point deadline) const;

   private:
    dai::MessageQueue& queue;
    std::vector<std::string> inputNames;
    dai::ImgFrame::Fsync expectedFsync;

    std::chrono::system_clock::time_point readTimestamp(const dai::MessageGroup& group, const std::string& name) const;
    GroupReadResult analyze(const dai::MessageGroup& group) const;
};

// Wait for the first validated group; the positive timeout starts on entry.
// Fail if no group arrives before the deadline.
GroupReadResult waitForFirstGroup(const GroupReader& reader, std::chrono::seconds timeout);

// Consume groups for the duration starting on entry, after startup. Zero skips warmup.
void consumeWarmup(const GroupReader& reader, std::chrono::seconds duration);

// Return the third consecutive group strictly within the positive spread threshold.
// Both adjacent median intervals must be positive and <= 1.5 / targetFps seconds; FPS must be finite and positive.
// Misalignment clears the run; an aligned group after an invalid interval starts a new run of one.
// The positive timeout starts on entry and is never restarted; fail if convergence times out.
// Check the initial candidate once before reading the queue; if aligned, it starts the run at one.
// Seed with the startup group only when warmup is disabled; otherwise discard that group.
// Only the returned group becomes measurement sample 0; earlier convergence groups are discarded.
GroupReadResult waitForConvergence(const GroupReader& reader,
                                  std::chrono::seconds timeout,
                                  std::chrono::system_clock::duration syncThreshold,
                                  float targetFps,
                                  std::optional<GroupReadResult> initialGroup = std::nullopt);

// Measure for a positive duration starting on entry, counting the convergence group once.
// Retain all groups and fail immediately on nonincreasing timestamps or gaps over 1.5 frame periods.
// Continuity starts at the convergence sample; a skipped interval is detected when the next group arrives.
// Fail on delivery silence reaching noProgressTimeout, including the trailing interval at measurement end.
std::vector<GroupReadResult> collectMeasurements(const GroupReader& reader,
                                               GroupReadResult firstGroup,
                                               std::chrono::seconds duration,
                                               float targetFps,
                                               std::chrono::seconds noProgressTimeout = std::chrono::seconds(1));

// Require at least 101 samples, report mean/p99/max spread, and check the mean and p99 limits.
void reportAndCheckStatistics(const std::vector<GroupReadResult>& samples, float targetFps, const FsyncTestParameters& parameters);

void setUpCameraSocket(dai::Pipeline& pipeline,
                       std::shared_ptr<dai::Device> device,
                       std::shared_ptr<dai::node::Sync> syncNode,
                       dai::CameraBoardSocket socket,
                       std::string& deviceName,
                       float targetFps,
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::vector<std::string>& inputNames);

void setUpIrLeds(std::shared_ptr<dai::Device> device);

void setupDevice(dai::DeviceInfo& deviceInfo,
                 dai::Pipeline& pipeline,
                 std::shared_ptr<dai::node::Sync> syncNode,
                 uint32_t &numMasters,
                 uint32_t &numSlaves,
                 std::vector<std::string>& inputNames,
                 float targetFps,
                 SyncType syncType,
                 std::optional<std::set<std::string>> &allowedSensors);

std::tuple<dai::Pipeline, std::shared_ptr<dai::node::Sync>, std::vector<std::string>> setupPipeline(
                   float targetFps,
                   struct FsyncTestParameters parameters);

int testSync(float targetFps, struct FsyncTestParameters parameters);
