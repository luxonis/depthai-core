// Regression for the host-side local-origin check of multi-device calibration.
//
// The host must admit a graph with the same rule the device firmware uses: the lowest-id socket on
// RVC2, the extrinsics chain root on RVC4. A RAM-only calibration whose chain root is not the lowest
// socket makes the two rules disagree, so the same graph must be accepted at startup (device check)
// and at runtime (host check first). One device is enough: the other graph endpoint is virtual.
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

std::vector<dai::MultiDeviceExtrinsics> graphWithLocalOrigin(const std::string& deviceId, dai::CameraBoardSocket localOrigin) {
    dai::MultiDeviceExtrinsics edge;
    edge.fromDeviceId = deviceId;
    edge.fromSocket = localOrigin;
    edge.extrinsics = dai::Extrinsics({{1.0f, 0.0f, 0.0f, kBridgeTranslationM}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
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

        lowestQueue = pipeline.create<dai::node::Camera>()->build(layout.lowest, std::nullopt, 10.0f)->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.0f)->createOutputQueue(4, false);
        rootQueue = pipeline.create<dai::node::Camera>()->build(layout.root, std::nullopt, 10.0f)->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.0f)->createOutputQueue(4, false);
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
