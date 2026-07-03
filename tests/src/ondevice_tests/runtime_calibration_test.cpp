#include <catch2/catch_all.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

namespace {

constexpr std::array<std::array<float, 3>, 3> kIdentityRotation = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};

std::vector<std::vector<float>> makeIdentityRotation() {
    return {{kIdentityRotation[0][0], kIdentityRotation[0][1], kIdentityRotation[0][2]},
            {kIdentityRotation[1][0], kIdentityRotation[1][1], kIdentityRotation[1][2]},
            {kIdentityRotation[2][0], kIdentityRotation[2][1], kIdentityRotation[2][2]}};
}

std::vector<std::vector<float>> makeIntrinsics() {
    return {{1000.0f, 0.0f, 640.0f}, {0.0f, 1000.0f, 400.0f}, {0.0f, 0.0f, 1.0f}};
}

struct StereoPairTestContext {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::CalibrationHandler originalCalibration = device->getCalibration();
    std::vector<dai::CameraFeatures> originalFeatures = device->getConnectedCameraFeatures();
    std::vector<dai::StereoPair> originalPairs = device->getStereoPairs();

    ~StereoPairTestContext() {
        try {
            device->setCalibration(originalCalibration);
            device->overrideCameraFeatures(originalFeatures);
        } catch(...) {
        }
    }

    void requireStereoPair() const {
        if(originalPairs.empty()) {
            SKIP("Device does not expose any stereo pairs");
        }
    }

    dai::StereoPair pairUnderTest() const {
        requireStereoPair();
        return originalPairs.front();
    }

    std::pair<dai::CameraBoardSocket, dai::CameraBoardSocket> featureOrderForPair() const {
        const auto pair = pairUnderTest();

        std::vector<dai::CameraBoardSocket> ordered;
        for(const auto& feature : originalFeatures) {
            if(feature.socket == pair.left || feature.socket == pair.right) {
                ordered.push_back(feature.socket);
            }
        }

        REQUIRE(ordered.size() == 2);
        return {ordered[0], ordered[1]};
    }

    void normalizePairFeatures(const std::string& sensorName = "OV9282", dai::CameraSensorType type = dai::CameraSensorType::MONO) {
        auto features = originalFeatures;
        const auto pair = pairUnderTest();
        for(auto& feature : features) {
            if(feature.socket == pair.left || feature.socket == pair.right) {
                feature.sensorName = sensorName;
                feature.width = 1280;
                feature.height = 800;
                feature.supportedTypes = {type};
            }
        }
        device->overrideCameraFeatures(features);
    }

    void applyCalibration(const dai::CalibrationHandler& calibration) const {
        device->setCalibration(calibration);
    }

    dai::CalibrationHandler calibrationWithPairExtrinsics(const std::vector<float>& translation) const {
        auto calibration = originalCalibration;
        const auto [socket1, socket2] = featureOrderForPair();
        const auto intrinsics = makeIntrinsics();

        calibration.setCameraIntrinsics(socket1, intrinsics, 1280, 800);
        calibration.setCameraIntrinsics(socket2, intrinsics, 1280, 800);
        calibration.setCameraExtrinsics(socket1, socket2, makeIdentityRotation(), translation, translation);
        return calibration;
    }

    bool pairPresent(const std::vector<dai::StereoPair>& pairs) const {
        const auto pair = pairUnderTest();
        return std::any_of(pairs.begin(), pairs.end(), [&](const dai::StereoPair& candidate) {
            return (candidate.left == pair.left && candidate.right == pair.right) || (candidate.left == pair.right && candidate.right == pair.left);
        });
    }

    const dai::StereoPair& requirePairPresent(const std::vector<dai::StereoPair>& pairs) const {
        const auto pair = pairUnderTest();
        const auto it = std::find_if(pairs.begin(), pairs.end(), [&](const dai::StereoPair& candidate) {
            return (candidate.left == pair.left && candidate.right == pair.right) || (candidate.left == pair.right && candidate.right == pair.left);
        });
        REQUIRE(it != pairs.end());
        return *it;
    }

    std::optional<dai::CameraBoardSocket> findThirdCalibratedSocket() const {
        const auto pair = pairUnderTest();
        for(const auto& feature : originalFeatures) {
            if(feature.socket == pair.left || feature.socket == pair.right) continue;
            if(originalCalibration.hasCameraCalibration(feature.socket)) {
                return feature.socket;
            }
        }
        return std::nullopt;
    }
};

}  // namespace

TEST_CASE("Test runtime calibration") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    float epsilon = std::numeric_limits<float>::epsilon();

    dai::Device device;

    dai::CalibrationHandler calibHandler, calibHandlerReadBack;

    dai::CameraBoardSocket cameraId = dai::CameraBoardSocket::CAM_B;
    std::vector<std::vector<float>> intrinsics = {{1000.0f, 0.0f, 320.0f}, {0.0f, 1000.0f, 240.0f}, {0.0f, 0.0f, 1.0f}};
    std::tuple<int, int> frameSize = {1280, 800};

    calibHandler.setCameraIntrinsics(cameraId, intrinsics, frameSize);
    device.setCalibration(calibHandler);

    try {
        calibHandlerReadBack = device.getCalibration();
        auto readBackIntrinsics = calibHandlerReadBack.getCameraIntrinsics(cameraId);

        for(size_t i = 0; i < intrinsics.size(); ++i) {
            for(size_t j = 0; j < intrinsics[i].size(); ++j) {
                REQUIRE(std::abs(intrinsics[i][j] - readBackIntrinsics[i][j]) < epsilon);
            }
        }

    } catch(const std::exception& e) {
        FAIL("Failed to read back intrinsics: " << e.what());
    }

    std::vector<float> distortionCoefficients;
    for(size_t i = 0; i < 14; ++i) {
        distortionCoefficients.push_back(i);
    }

    calibHandler.setDistortionCoefficients(cameraId, distortionCoefficients);
    device.setCalibration(calibHandler);

    try {
        calibHandlerReadBack = device.getCalibration();
        auto readBackDistortionCoefficients = calibHandlerReadBack.getDistortionCoefficients(cameraId);

        REQUIRE(readBackDistortionCoefficients.size() == distortionCoefficients.size());
        for(size_t i = 0; i < distortionCoefficients.size(); ++i) {
            REQUIRE(std::abs(distortionCoefficients[i] - readBackDistortionCoefficients[i]) < epsilon);
        }

    } catch(const std::exception& e) {
        FAIL("Failed to read back distortion coefficients: " << e.what());
    }
}

TEST_CASE("Test device setCalibration before pipeline build") {
    dai::Pipeline p;
    auto camQ = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();
    dai::CalibrationHandler calibHandler = p.getDefaultDevice()->getCalibration();
    calibHandler.validateCalibrationHandler();
    calibHandler.setDeviceName("test_device_name");
    p.getDefaultDevice()->setCalibration(calibHandler);

    auto calibPreBuild = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPreBuild.getEepromData().deviceName == "test_device_name");

    p.build();

    auto calibPostBuild = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostBuild.getEepromData().deviceName == "test_device_name");

    p.start();

    auto calibPostStart = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostStart.getEepromData().deviceName == "test_device_name");

    p.stop();
}

TEST_CASE("Test pipeline setCalibration before pipeline build") {
    dai::Pipeline p;
    auto camQ = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();
    dai::CalibrationHandler calibHandler = p.getDefaultDevice()->getCalibration();
    calibHandler.validateCalibrationHandler();
    calibHandler.setDeviceName("test_device_name");
    p.setCalibrationData(calibHandler);

    p.start();

    auto calibPostStart = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostStart.getEepromData().deviceName == "test_device_name");

    p.stop();
}

TEST_CASE("getStereoPairs filters out pairs with z-dominant translation") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();
    ctx.normalizePairFeatures();
    ctx.applyCalibration(ctx.calibrationWithPairExtrinsics({1.0f, 1.0f, 5.0f}));

    REQUIRE_FALSE(ctx.pairPresent(ctx.device->getStereoPairs()));
}

TEST_CASE("getStereoPairs filters out pairs when sensors differ") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();

    auto features = ctx.originalFeatures;
    const auto pair = ctx.pairUnderTest();
    bool first = true;
    for(auto& feature : features) {
        if(feature.socket == pair.left || feature.socket == pair.right) {
            feature.sensorName = first ? "OV9282" : "IMX378";
            feature.width = 1280;
            feature.height = 800;
            feature.supportedTypes = {dai::CameraSensorType::MONO};
            first = false;
        }
    }
    ctx.device->overrideCameraFeatures(features);
    ctx.applyCalibration(ctx.calibrationWithPairExtrinsics({5.0f, 1.0f, 0.0f}));

    REQUIRE_FALSE(ctx.pairPresent(ctx.device->getStereoPairs()));
}

TEST_CASE("getStereoPairs filters out pairs when one camera is not stereo capable") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();

    auto features = ctx.originalFeatures;
    const auto pair = ctx.pairUnderTest();
    for(auto& feature : features) {
        if(feature.socket == pair.left || feature.socket == pair.right) {
            feature.sensorName = "OV9282";
            feature.width = 1280;
            feature.height = 800;
            feature.supportedTypes = {feature.socket == pair.left ? dai::CameraSensorType::THERMAL : dai::CameraSensorType::MONO};
        }
    }
    ctx.device->overrideCameraFeatures(features);
    ctx.applyCalibration(ctx.calibrationWithPairExtrinsics({5.0f, 1.0f, 0.0f}));

    REQUIRE_FALSE(ctx.pairPresent(ctx.device->getStereoPairs()));
}

TEST_CASE("getStereoPairs filters out pairs when calibration is missing for one socket") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();
    ctx.normalizePairFeatures();

    auto eeprom = ctx.originalCalibration.getEepromData();
    eeprom.cameraData.erase(ctx.pairUnderTest().left);
    ctx.applyCalibration(dai::CalibrationHandler(eeprom));

    REQUIRE_FALSE(ctx.pairPresent(ctx.device->getStereoPairs()));
}

TEST_CASE("getStereoPairs filters out pairs when extrinsics link is missing") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();
    ctx.normalizePairFeatures();

    auto eeprom = ctx.originalCalibration.getEepromData();
    const auto [socket1, socket2] = ctx.featureOrderForPair();
    auto& camera = eeprom.cameraData.at(socket1);
    camera.extrinsics.toCameraSocket = dai::CameraBoardSocket::AUTO;
    camera.extrinsics.rotationMatrix = makeIdentityRotation();
    camera.extrinsics.translation = {5.0f, 1.0f, 0.0f};
    camera.extrinsics.specTranslation = {5.0f, 1.0f, 0.0f};
    ctx.applyCalibration(dai::CalibrationHandler(eeprom));

    REQUIRE_FALSE(ctx.pairPresent(ctx.device->getStereoPairs()));
}

TEST_CASE("getStereoPairs builds a horizontal stereo pair with expected ordering") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();
    ctx.normalizePairFeatures();
    const auto [socket1, socket2] = ctx.featureOrderForPair();
    ctx.applyCalibration(ctx.calibrationWithPairExtrinsics({5.0f, 1.0f, 0.0f}));

    const auto pairs = ctx.device->getStereoPairs();
    const auto& pair = ctx.requirePairPresent(pairs);
    REQUIRE_FALSE(pair.isVertical);
    REQUIRE(pair.baseline == Catch::Approx(5.0f));
    REQUIRE(pair.left == socket2);
    REQUIRE(pair.right == socket1);
}

TEST_CASE("getStereoPairs builds a vertical stereo pair with expected ordering") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();
    ctx.normalizePairFeatures();
    const auto [socket1, socket2] = ctx.featureOrderForPair();
    ctx.applyCalibration(ctx.calibrationWithPairExtrinsics({1.0f, -5.0f, 0.0f}));

    const auto pairs = ctx.device->getStereoPairs();
    const auto& pair = ctx.requirePairPresent(pairs);
    REQUIRE(pair.isVertical);
    REQUIRE(pair.baseline == Catch::Approx(5.0f));
    REQUIRE(pair.left == socket1);
    REQUIRE(pair.right == socket2);
}

TEST_CASE("getStereoPairs keeps only the linked pair when a third camera has no extrinsics link") {
    StereoPairTestContext ctx;
    ctx.requireStereoPair();

    const auto thirdSocket = ctx.findThirdCalibratedSocket();
    if(!thirdSocket.has_value()) {
        SKIP("Device does not expose a third calibrated camera");
    }

    auto features = ctx.originalFeatures;
    const auto pair = ctx.pairUnderTest();
    for(auto& feature : features) {
        if(feature.socket == pair.left || feature.socket == pair.right || feature.socket == *thirdSocket) {
            feature.sensorName = "OV9282";
            feature.width = 1280;
            feature.height = 800;
            feature.supportedTypes = {dai::CameraSensorType::MONO};
        }
    }
    ctx.device->overrideCameraFeatures(features);

    auto eeprom = ctx.originalCalibration.getEepromData();
    const auto [socket1, socket2] = ctx.featureOrderForPair();
    const auto intrinsics = makeIntrinsics();
    eeprom.cameraData.at(socket1).intrinsicMatrix = intrinsics;
    eeprom.cameraData.at(socket1).width = 1280;
    eeprom.cameraData.at(socket1).height = 800;
    eeprom.cameraData.at(socket1).specHfovDeg = 70.0f;
    eeprom.cameraData.at(socket2).intrinsicMatrix = intrinsics;
    eeprom.cameraData.at(socket2).width = 1280;
    eeprom.cameraData.at(socket2).height = 800;
    eeprom.cameraData.at(socket2).specHfovDeg = 70.0f;
    eeprom.cameraData.at(*thirdSocket).intrinsicMatrix = intrinsics;
    eeprom.cameraData.at(*thirdSocket).width = 1280;
    eeprom.cameraData.at(*thirdSocket).height = 800;
    eeprom.cameraData.at(*thirdSocket).specHfovDeg = 70.0f;
    eeprom.cameraData.at(socket1).extrinsics.rotationMatrix = makeIdentityRotation();
    eeprom.cameraData.at(socket1).extrinsics.translation = {5.0f, 1.0f, 0.0f};
    eeprom.cameraData.at(socket1).extrinsics.specTranslation = {5.0f, 1.0f, 0.0f};
    eeprom.cameraData.at(socket1).extrinsics.toCameraSocket = socket2;
    eeprom.cameraData.at(*thirdSocket).extrinsics.rotationMatrix = makeIdentityRotation();
    eeprom.cameraData.at(*thirdSocket).extrinsics.translation = {5.0f, 1.0f, 0.0f};
    eeprom.cameraData.at(*thirdSocket).extrinsics.specTranslation = {5.0f, 1.0f, 0.0f};
    eeprom.cameraData.at(*thirdSocket).extrinsics.toCameraSocket = dai::CameraBoardSocket::AUTO;
    ctx.applyCalibration(dai::CalibrationHandler(eeprom));

    const auto pairs = ctx.device->getStereoPairs();
    const auto& linkedPair = ctx.requirePairPresent(pairs);
    REQUIRE_FALSE(linkedPair.isVertical);
    REQUIRE(linkedPair.baseline == Catch::Approx(5.0f));

    const bool thirdSocketUsed = std::any_of(pairs.begin(), pairs.end(), [&](const dai::StereoPair& candidate) {
        return candidate.left == *thirdSocket || candidate.right == *thirdSocket;
    });
    REQUIRE_FALSE(thirdSocketUsed);
}
