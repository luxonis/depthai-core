#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fsync_ptp_test_utils.hpp"

TEST_CASE("Test Multi-device PTP frame sync with different FPS values", "[ptp]") {
    // auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 60.0f, 120.0f, 240.0f, 300.0f, 600.0f);
    // 60 FPS does not work as of 1.30.1
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f, 45.0f);
    CAPTURE(fps);
    struct FsyncTestParameters parameters {};
    parameters.syncThresholdSec = 1/(2*fps); // lower this limit when we have better accuracy for timestamps
    parameters.testDurationSec = 180;
    parameters.recvAllTimeoutSec = 15;
    parameters.initialSyncTimeoutSec = 60;
    parameters.initialTimeoutSec = 60;
    parameters.deltaMeanThreshold = 1e-3;
    parameters.deltaP99Threshold = 2e-3;
    parameters.syncType = SyncType::PTP;
    testFsync(fps, parameters);
}
