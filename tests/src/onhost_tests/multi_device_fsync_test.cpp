#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fsync_ptp_test_utils.hpp"

namespace {

void runExternalSyncTest(float fps) {
    struct FsyncTestParameters parameters {};
    parameters.syncThresholdSec = 1 / (2 * fps);  // lower this limit when we have better accuracy for timestamps
    parameters.testDurationSec = 180;
    parameters.recvAllTimeoutSec = 10;
    parameters.initialSyncTimeoutSec = 4;
    parameters.initialTimeoutSec = 0;
    parameters.deltaMeanThreshold = 1e-3;
    parameters.deltaP99Threshold = 2e-3;
    parameters.syncType = SyncType::EXTERNAL;
    testFsync(fps, parameters);
}

}

TEST_CASE("Test Multi-device external frame sync with at 10 FPS", "[fsync][fps-10]") {
    runExternalSyncTest(10.0f);
}

TEST_CASE("Test Multi-device external frame sync with at 13 FPS", "[fsync][fps-13]") {
    runExternalSyncTest(13.0f);
}

TEST_CASE("Test Multi-device external frame sync with at 18.5 FPS", "[fsync][fps-18.5]") {
    runExternalSyncTest(18.5f);
}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS", "[fsync][fps-30]") {
    runExternalSyncTest(30.0f);
}

TEST_CASE("Test Multi-device external frame sync with at 45 FPS", "[fsync][fps-45]") {
    runExternalSyncTest(45.0f);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60]") {
    runExternalSyncTest(60.0f);
}