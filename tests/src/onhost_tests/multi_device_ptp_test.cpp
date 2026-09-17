#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fsync_ptp_test_utils.hpp"

namespace {

void runExternalSyncTest(float fps) {
    struct FsyncTestParameters parameters {};
    parameters.syncThresholdSec = 1 / (2 * fps);  // lower this limit when we have better accuracy for timestamps
    parameters.testDurationSec = 180;
    parameters.recvAllTimeoutSec = 15;
    parameters.initialSyncTimeoutSec = 60;
    parameters.initialTimeoutSec = 60;
    parameters.deltaMeanThreshold = 1e-3;
    parameters.deltaP99Threshold = 2e-3;
    parameters.syncType = SyncType::PTP;
    testFsync(fps, parameters);
}

}

TEST_CASE("Test Multi-device PTP frame sync with at 10 FPS", "[ptp][fps-10]") {
    runExternalSyncTest(10.0f);
}

TEST_CASE("Test Multi-device PTP frame sync with at 13 FPS", "[ptp][fps-13]") {
    runExternalSyncTest(13.0f);
}

TEST_CASE("Test Multi-device PTP frame sync with at 18.5 FPS", "[ptp][fps-18.5]") {
    runExternalSyncTest(18.5f);
}

TEST_CASE("Test Multi-device PTP frame sync with at 30 FPS", "[ptp][fps-30]") {
    runExternalSyncTest(30.0f);
}

TEST_CASE("Test Multi-device PTP frame sync with at 45 FPS", "[ptp][fps-45]") {
    runExternalSyncTest(45.0f);
}

TEST_CASE("Test Multi-device PTP frame sync with at 60 FPS", "[ptp][fps-60]") {
    runExternalSyncTest(60.0f);
}
