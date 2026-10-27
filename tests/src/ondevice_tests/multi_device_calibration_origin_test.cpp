// Regressions for multi-device calibration on one RVC2 device. The other graph endpoint is virtual.
//
// 1. Host-side local-origin check: the host must admit a graph with the same rule the device firmware
//    uses, the lowest-id socket on RVC2 and the extrinsics chain root on RVC4. A RAM-only calibration
//    whose chain root is not the lowest socket makes the two rules disagree, so the same graph must be
//    accepted at startup (device check) and at runtime (host check first).
// 2. Linked StereoDepth alignment: the camera feeding inputAlignTo publishes an already rebased pose
//    and StereoDepth caches the alignment transformation, so depth aligned through inputAlignTo must
//    still follow a graph that is replaced or removed at runtime.
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <thread>

#include "depthai/depthai.hpp"

namespace {

constexpr const char* kVirtualDeviceId = "!virtual-origin";  // Sorts below any serial, so it becomes the component origin.
constexpr float kPairBaselineCm = 7.5f;
constexpr float kBridgeTranslationM = 1.0f;

struct Layout {
    dai::CameraBoardSocket lowest;  // RVC2 local origin
    dai::CameraBoardSocket root;    // RVC4 / host chain root
};

// Two connected sockets, calibrated as the chain lowest -> root -> AUTO.
Layout pickLayout(const std::shared_ptr<dai::Device>& device) {
    auto sockets = device->getConnectedCameras();
    if(sockets.size() < 2) {
        SKIP("The test needs two connected cameras");
    }
    std::sort(sockets.begin(), sockets.end(), [](dai::CameraBoardSocket a, dai::CameraBoardSocket b) { return static_cast<int>(a) < static_cast<int>(b); });
    return {sockets[0], sockets[1]};
}

dai::CalibrationHandler synthesizeCalibration(const Layout& layout) {
    const std::vector<std::vector<float>> intrinsics = {{1000.0f, 0.0f, 640.0f}, {0.0f, 1000.0f, 400.0f}, {0.0f, 0.0f, 1.0f}};
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const std::vector<float> zero = {0.0f, 0.0f, 0.0f};
    const std::vector<float> baseline = {kPairBaselineCm, 0.0f, 0.0f};

    dai::CalibrationHandler calibration;
    for(const auto socket : {layout.lowest, layout.root}) {
        calibration.setCameraIntrinsics(socket, intrinsics, 1280, 800);
        calibration.setDistortionCoefficients(socket, std::vector<float>(14, 0.0f));
    }
    calibration.setCameraExtrinsics(layout.root, dai::CameraBoardSocket::AUTO, identity, zero, zero);
    calibration.setCameraExtrinsics(layout.lowest, layout.root, identity, baseline, baseline);
    return calibration;
}

std::vector<dai::MultiDeviceExtrinsics> graphWithLocalOrigin(const std::string& deviceId,
                                                             dai::CameraBoardSocket localOrigin,
                                                             float bridgeTranslationM = kBridgeTranslationM) {
    dai::MultiDeviceExtrinsics edge;
    edge.fromDeviceId = deviceId;
    edge.fromSocket = localOrigin;
    edge.extrinsics = dai::Extrinsics({{1.0f, 0.0f, 0.0f, bridgeTranslationM}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
                                      dai::CameraBoardSocket::CAM_A,
                                      dai::LengthUnit::METER);
    edge.extrinsics.toDeviceId = kVirtualDeviceId;
    return {edge};
}

std::shared_ptr<dai::ImgFrame> waitForFrame(const std::shared_ptr<dai::MessageQueue>& queue, const std::string& toDeviceId, dai::CameraBoardSocket toSocket) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while(std::chrono::steady_clock::now() < deadline) {
        auto frame = queue->tryGet<dai::ImgFrame>();
        if(!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        const auto extrinsics = frame->getTransformation().getExtrinsics();
        if(extrinsics.toDeviceId == toDeviceId && extrinsics.toCameraSocket == toSocket) {
            return frame;
        }
    }
    FAIL("No frame with extrinsics to " << toDeviceId << " / " << static_cast<int>(toSocket) << " within 15 s");
    return nullptr;
}

dai::Point3f translationCm(const std::shared_ptr<dai::ImgFrame>& frame) {
    const auto t = frame->getTransformation().getExtrinsics().getTranslationVector(false, dai::LengthUnit::CENTIMETER);
    return {t[0], t[1], t[2]};
}

float distance(const dai::Point3f& a, const dai::Point3f& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// Waits until a frame carries the expected pose: the same target device and socket and a translation
// within 1 mm. Two graphs with the same target differ only by their translation, so the ids alone do
// not tell whether a replaced graph has been applied.
void expectPose(const std::shared_ptr<dai::MessageQueue>& queue,
                const std::string& toDeviceId,
                dai::CameraBoardSocket toSocket,
                const dai::Point3f& translationCmExpected) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    std::string lastSeen = "no frame";
    while(std::chrono::steady_clock::now() < deadline) {
        auto frame = queue->tryGet<dai::ImgFrame>();
        if(!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        const auto extrinsics = frame->getTransformation().getExtrinsics();
        const auto t = translationCm(frame);
        if(extrinsics.toDeviceId == toDeviceId && extrinsics.toCameraSocket == toSocket && distance(t, translationCmExpected) < 0.1f) {
            return;
        }
        lastSeen = extrinsics.toDeviceId + " / " + std::to_string(static_cast<int>(extrinsics.toCameraSocket)) + " at (" + std::to_string(t.x) + ", "
                   + std::to_string(t.y) + ", " + std::to_string(t.z) + ") cm";
    }
    FAIL("No frame with extrinsics to " << toDeviceId << " / " << static_cast<int>(toSocket) << " at (" << translationCmExpected.x << ", "
                                        << translationCmExpected.y << ", " << translationCmExpected.z << ") cm within 15 s; last seen " << lastSeen);
}

// A PoE RVC2 device reboots after a pipeline closes and needs a while to come back for the next case.
dai::Pipeline openPipeline() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
    while(true) {
        try {
            return dai::Pipeline();
        } catch(const std::exception&) {
            if(std::chrono::steady_clock::now() >= deadline) {
                throw;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
}

struct Rig {
    dai::Pipeline pipeline = openPipeline();
    std::shared_ptr<dai::Device> device;
    Layout layout;
    std::string deviceId;
    std::shared_ptr<dai::MessageQueue> lowestQueue;
    std::shared_ptr<dai::MessageQueue> rootQueue;

    Rig() {
        device = pipeline.getDefaultDevice();
        REQUIRE(device);
        if(device->getPlatform() != dai::Platform::RVC2) {
            SKIP("The lowest-socket local origin is the RVC2 rule; RVC4 publishes the chain root the host already uses");
        }
        layout = pickLayout(device);
        deviceId = device->getDeviceId();
        // RAM only: the EEPROM is never written.
        device->setCalibration(synthesizeCalibration(layout));

        lowestQueue = pipeline.create<dai::node::Camera>()
                          ->build(layout.lowest, std::nullopt, 10.0f)
                          ->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.0f)
                          ->createOutputQueue(4, false);
        rootQueue = pipeline.create<dai::node::Camera>()
                        ->build(layout.root, std::nullopt, 10.0f)
                        ->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.0f)
                        ->createOutputQueue(4, false);
    }

    std::vector<dai::MultiDeviceExtrinsics> graphFor(dai::CameraBoardSocket socket) const {
        return graphWithLocalOrigin(deviceId, socket);
    }

    void expectRebasedToVirtualOrigin() {
        auto lowest = waitForFrame(lowestQueue, kVirtualDeviceId, dai::CameraBoardSocket::CAM_A);
        auto root = waitForFrame(rootQueue, kVirtualDeviceId, dai::CameraBoardSocket::CAM_A);
        // The lowest socket is the device's local origin, so its pose is the bridge itself.
        const dai::Point3f bridge{kBridgeTranslationM * 100.0f, 0.0f, 0.0f};
        REQUIRE(distance(translationCm(lowest), bridge) < 0.1f);
        // The root socket sits one baseline away from the lowest socket, on either side.
        REQUIRE(std::abs(distance(translationCm(root), bridge) - kPairBaselineCm) < 0.1f);
    }

    void expectLocalOrigin() {
        waitForFrame(lowestQueue, deviceId, layout.lowest);
        auto root = waitForFrame(rootQueue, deviceId, layout.lowest);
        REQUIRE(std::abs(distance(translationCm(root), {0.0f, 0.0f, 0.0f}) - kPairBaselineCm) < 0.1f);
    }
};

// StereoDepth on the device's own calibration, depth aligned through a linked inputAlignTo camera.
struct StereoRig {
    dai::Pipeline pipeline = openPipeline();
    std::shared_ptr<dai::Device> device;
    std::string deviceId;
    dai::CameraBoardSocket localOrigin = dai::CameraBoardSocket::AUTO;  // lowest-id calibrated socket, the RVC2 rule
    dai::CameraBoardSocket alignSocket = dai::CameraBoardSocket::AUTO;
    dai::Point3f localTranslationCm;  // alignSocket -> localOrigin, what the depth frames carry without a graph
    std::shared_ptr<dai::MessageQueue> depthQueue;

    StereoRig() {
        device = pipeline.getDefaultDevice();
        REQUIRE(device);
        if(device->getPlatform() != dai::Platform::RVC2) {
            SKIP("Cached alignment transformations are an RVC2 StereoDepth trait; RVC4 re-reads inputAlignTo per frame");
        }
        const auto pairs = device->getStereoPairs();
        if(pairs.empty()) {
            SKIP("The test needs a calibrated stereo pair");
        }
        const auto left = pairs.front().left;
        const auto right = pairs.front().right;
        // Prefer a third camera so the local alignment pose is not trivially the stereo baseline.
        alignSocket = right;
        for(const auto socket : device->getConnectedCameras()) {
            if(socket != left && socket != right) {
                alignSocket = socket;
                break;
            }
        }

        const auto calibration = device->readCalibration();
        const auto& cameraData = calibration.getEepromData().cameraData;
        REQUIRE(!cameraData.empty());
        localOrigin = std::min_element(cameraData.begin(), cameraData.end(), [](const auto& a, const auto& b) {
                          return static_cast<int>(a.first) < static_cast<int>(b.first);
                      })->first;
        const auto t = calibration.getCameraTranslationVector(alignSocket, localOrigin, false);
        localTranslationCm = {t[0], t[1], t[2]};
        deviceId = device->getDeviceId();

        auto* leftOut =
            pipeline.create<dai::node::Camera>()->build(left, std::nullopt, 10.0f)->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, 10.0f);
        auto* rightOut =
            pipeline.create<dai::node::Camera>()->build(right, std::nullopt, 10.0f)->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, 10.0f);
        auto* alignOut = pipeline.create<dai::node::Camera>()
                             ->build(alignSocket, std::nullopt, 10.0f)
                             ->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.0f);
        auto stereo = pipeline.create<dai::node::StereoDepth>()->build(*leftOut, *rightOut);
        alignOut->link(stereo->inputAlignTo);
        depthQueue = stereo->depth.createOutputQueue(4, false);
    }

    std::vector<dai::MultiDeviceExtrinsics> graphWithBridge(float bridgeTranslationM) const {
        return graphWithLocalOrigin(deviceId, localOrigin, bridgeTranslationM);
    }

    // The bridge has identity rotation, so the rebased pose is the local pose shifted by the bridge.
    void expectDepthRebased(float bridgeTranslationM) {
        const dai::Point3f expected{localTranslationCm.x + bridgeTranslationM * 100.0f, localTranslationCm.y, localTranslationCm.z};
        expectPose(depthQueue, kVirtualDeviceId, dai::CameraBoardSocket::CAM_A, expected);
    }

    void expectDepthLocal() {
        expectPose(depthQueue, deviceId, localOrigin, localTranslationCm);
    }
};

}  // namespace

TEST_CASE("Multi-device calibration graph naming the RVC2 local origin is accepted at startup and at runtime") {
    Rig rig;
    const auto graph = rig.graphFor(rig.layout.lowest);

    // Startup: only the device validates, with its own rule.
    rig.pipeline.setMultiDeviceCalibration(graph);
    REQUIRE_NOTHROW(rig.pipeline.start());
    rig.expectRebasedToVirtualOrigin();

    // Runtime: the host validates first and must agree with the device.
    REQUIRE_NOTHROW(rig.pipeline.setMultiDeviceCalibration(graph));
    rig.expectRebasedToVirtualOrigin();

    // The chain root is not what RVC2 frames publish, so that graph is rejected with the rule named.
    REQUIRE_THROWS_WITH(rig.pipeline.setMultiDeviceCalibration(rig.graphFor(rig.layout.root)), Catch::Matchers::ContainsSubstring("lowest-id socket"));
    REQUIRE(rig.pipeline.getMultiDeviceCalibration().has_value());
    rig.expectRebasedToVirtualOrigin();

    REQUIRE_NOTHROW(rig.pipeline.clearMultiDeviceCalibration());
    rig.expectLocalOrigin();

    rig.pipeline.stop();
    rig.pipeline.wait();
}

TEST_CASE("Multi-device calibration graph naming the chain root is rejected by an RVC2 device at startup") {
    Rig rig;
    rig.pipeline.setMultiDeviceCalibration(rig.graphFor(rig.layout.root));
    REQUIRE_THROWS_WITH(rig.pipeline.start(), Catch::Matchers::ContainsSubstring("lowest-id socket"));
}

TEST_CASE("Depth aligned through a linked inputAlignTo follows a multi-device graph replaced or removed at runtime") {
    StereoRig rig;

    // StereoDepth reads the alignment pose while the first graph is installed, so the camera hands it an
    // already rebased pose. The firmware must not cache that pose as is.
    rig.pipeline.setMultiDeviceCalibration(rig.graphWithBridge(1.0f));
    REQUIRE_NOTHROW(rig.pipeline.start());
    rig.expectDepthRebased(1.0f);

    REQUIRE_NOTHROW(rig.pipeline.setMultiDeviceCalibration(rig.graphWithBridge(2.0f)));
    rig.expectDepthRebased(2.0f);

    REQUIRE_NOTHROW(rig.pipeline.clearMultiDeviceCalibration());
    rig.expectDepthLocal();

    REQUIRE_NOTHROW(rig.pipeline.setMultiDeviceCalibration(rig.graphWithBridge(1.0f)));
    rig.expectDepthRebased(1.0f);

    rig.pipeline.stop();
    rig.pipeline.wait();
}
