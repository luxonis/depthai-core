#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fsync_ptp_test_utils.hpp"

namespace {

struct FsyncTestParameters getDefaultParameters() {
    struct FsyncTestParameters parameters {};
    parameters.measurementDurationSec = 10;
    parameters.firstGroupTimeoutSec = 10;
    parameters.syncAcquisitionTimeoutSec = 1;
    parameters.warmupDurationSec = 0;
    parameters.deltaMeanThreshold = 2e-3;
    parameters.deltaP99Threshold = 2.5e-3;
    parameters.syncThresholdSec = 2.5e-3;
    parameters.syncType = SyncType::EXTERNAL;
    parameters.expectedDevices = 4;
    return parameters;
}

}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS on OV9282 sensors", "[fsync][fps-30][ov9282]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    // p.measurementDurationSec = 60;
    p.allowedSensors = std::set<std::string>{"OV9282"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS on IMX586 sensors", "[fsync][fps-30][imx586]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    // p.measurementDurationSec = 60;
    p.allowedSensors = std::set<std::string>{"IMX586"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 10 FPS", "[fsync][fps-10][ov9282][imx586][og05]") {
    float fps = 10.0f;
    auto p = getDefaultParameters();
    // Keep the 101-sample minimum comfortably inside the measurement window at 10 FPS.
    p.measurementDurationSec = 15;
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 13 FPS", "[fsync][fps-13][ov9282][imx586][og05]") {
    float fps = 13.0f;
    auto p = getDefaultParameters();
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 18.5 FPS", "[fsync][fps-18.5][ov9282][imx586][og05]") {
    float fps = 18.5f;
    auto p = getDefaultParameters();
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 30 FPS", "[fsync][fps-30][ov9282][imx586][og05]") {
    float fps = 30.0f;
    auto p = getDefaultParameters();
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 45 FPS", "[fsync][fps-45][ov9282][imx586][og05]") {
    float fps = 45.0f;
    auto p = getDefaultParameters();
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][ov9282][imx586][og05]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][ov9282][og05]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.allowedSensors = std::set<std::string>{"OV9282", "OG05B10"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][ov9282][imx586]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.allowedSensors = std::set<std::string>{"OV9282", "IMX586"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][og05][imx586]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.allowedSensors = std::set<std::string>{"OG05B10", "IMX586"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][imx586]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.allowedSensors = std::set<std::string>{"IMX586"};
    testSync(fps, p);
}

TEST_CASE("Test Multi-device external frame sync with at 60 FPS", "[fsync][fps-60][ov9282]") {
    float fps = 60.0f;
    auto p = getDefaultParameters();
    p.allowedSensors = std::set<std::string>{"OV9282"};
    testSync(fps, p);
}
