#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fsync_ptp_test_utils.hpp"

namespace {

struct FsyncTestParameters getDefaultParameters() {
    struct FsyncTestParameters parameters {};
    parameters.totalRunDurationSec = 180;
    parameters.firstGroupTimeoutSec = 10;
    parameters.syncAcquisitionTimeoutSec = 60;
    parameters.warmupDurationSec = 60;
    parameters.deltaMeanThreshold = 1e-3;
    parameters.deltaP99Threshold = 2e-3;
    parameters.syncType = SyncType::PTP;
    parameters.expectedDevices = 4;
    return parameters;
}

}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS on OV9282 sensors", "[ptp][fps-30][ov9282]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    p.totalRunDurationSec = 120;
    p.allowedSensors = std::set<std::string>{"OV9282"};
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS on IMX586 sensors", "[ptp][fps-30][imx586]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    p.totalRunDurationSec = 120;
    p.allowedSensors = std::set<std::string>{"IMX586"};
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 10 FPS", "[ptp][fps-10]") {
    float fps = 10.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 13 FPS", "[ptp][fps-13]") {
    float fps = 13.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 18.5 FPS", "[ptp][fps-18.5]") {
    float fps = 18.5f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS", "[ptp][fps-30]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 45 FPS", "[ptp][fps-45]") {
    float fps = 45.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[ptp][fps-60]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.syncThresholdSec = 1 / (2 * fps);
    testFsync(fps, p);
}