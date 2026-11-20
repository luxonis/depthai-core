#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace {

constexpr auto kFrameTimeout = std::chrono::seconds(10);
constexpr auto kReportTimeout = std::chrono::seconds(20);

bool hasTofSensor(dai::Device& device) {
    for(const auto& features : device.getConnectedCameraFeatures()) {
        for(const auto type : features.supportedTypes) {
            if(type == dai::CameraSensorType::TOF) return true;
        }
    }
    return false;
}

// Opens the first device once and skips the test when it has no ToF sensor.
// Later tests reuse the result: opening again right away finds the device still
// resetting after the previous test.
dai::DeviceInfo requireTofDevice() {
    static const std::optional<dai::DeviceInfo> tofDevice = []() -> std::optional<dai::DeviceInfo> {
        dai::Device device;
        if(!hasTofSensor(device)) return std::nullopt;
        // Keep the name too: a PoE device outside the host's subnet is found by its IP.
        auto info = device.getDeviceInfo();
        info.state = X_LINK_ANY_STATE;
        return info;
    }();
    if(!tofDevice) {
        SKIP("Requires a device with a ToF sensor.");
    }
    return *tofDevice;
}

bool isAvailable(const dai::DeviceInfo& info) {
    for(const auto& device : dai::XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, true, 50)) {
        if(device.deviceId == info.deviceId && device.state != X_LINK_BOOTED) return true;
    }
    return false;
}

// A closed device resets before it can be opened again: wait until it is
// available. A PoE device takes 20-30 s to reboot.
void waitUntilAvailable(const dai::DeviceInfo& info) {
    const auto start = std::chrono::steady_clock::now();
    while(!isAvailable(info) && std::chrono::steady_clock::now() - start < std::chrono::seconds(60)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

std::shared_ptr<dai::ImgFrame> requireFrame(dai::MessageQueue& queue, const std::string& name) {
    bool timedOut = false;
    auto frame = queue.get<dai::ImgFrame>(kFrameTimeout, timedOut);
    INFO("output: " << name);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    return frame;
}

void requireConsistentFrame(const dai::ImgFrame& frame, const std::string& name) {
    INFO("output: " << name << ", " << frame.getWidth() << "x" << frame.getHeight() << ", stride " << frame.getStride() << ", " << frame.getData().size()
                    << " bytes");
    REQUIRE(frame.getWidth() > 0);
    REQUIRE(frame.getHeight() > 0);
    REQUIRE(frame.getStride() >= frame.getWidth());
    REQUIRE(frame.getData().size() == static_cast<size_t>(frame.getStride()) * frame.getHeight());
}

}  // namespace

TEST_CASE("ToF outputs stream frames of the depth size", "[ToF]") {
    const auto info = requireTofDevice();
    waitUntilAvailable(info);
    auto device = std::make_shared<dai::Device>(info);
    const bool isRvc2 = device->getPlatform() == dai::Platform::RVC2;
    dai::Pipeline pipeline{device};

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, 30.0f);

    // Outputs computed from depth, so each has the depth size.
    std::map<std::string, std::shared_ptr<dai::MessageQueue>> depthSized{
        {"amplitude", tof->amplitude.createOutputQueue(4, false)},
        {"intensity", tof->intensity.createOutputQueue(4, false)},
        {"confidence", tof->confidence.createOutputQueue(4, false)},
        {"phase", tof->phase.createOutputQueue(4, false)},
    };
    if(isRvc2) {
        // On RVC4 rawDepth is an unconnected placeholder.
        depthSized.emplace("rawDepth", tof->rawDepth.createOutputQueue(4, false));
    }
    auto depthQueue = tof->depth.createOutputQueue(4, false);
    auto rawQueue = isRvc2 ? tof->raw.createOutputQueue(4, false) : nullptr;

    pipeline.start();

    auto depth = requireFrame(*depthQueue, "depth");
    requireConsistentFrame(*depth, "depth");
    REQUIRE(depth->getType() == dai::ImgFrame::Type::RAW16);
    REQUIRE(depth->getStride() == depth->getWidth() * 2);

    for(const auto& [name, queue] : depthSized) {
        auto frame = requireFrame(*queue, name);
        requireConsistentFrame(*frame, name);
        INFO("output: " << name);
        REQUIRE(frame->getWidth() == depth->getWidth());
        REQUIRE(frame->getHeight() == depth->getHeight());
    }

    if(rawQueue) {
        // The raw sensor frame holds all subframes that one depth frame is computed from.
        auto raw = requireFrame(*rawQueue, "raw");
        requireConsistentFrame(*raw, "raw");
        REQUIRE(raw->getHeight() > depth->getHeight());
    }

    pipeline.stop();
}

TEST_CASE("ToF depth streams at the requested FPS", "[ToF]") {
    const auto info = requireTofDevice();

    for(const float fps : {30.0f, 20.0f, 15.0f, 10.0f, 5.0f}) {
        INFO("requested FPS: " << fps);
        waitUntilAvailable(info);
        dai::Pipeline pipeline{std::make_shared<dai::Device>(info)};

        auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, fps);

        // Measure on the device: on RVC2 ToF::depth comes from host ImageFilters.
        auto benchmark = pipeline.create<dai::node::BenchmarkIn>();
        benchmark->setRunOnHost(false);
        benchmark->logReportsAsWarnings(false);
        benchmark->sendReportEveryNMessages(static_cast<uint32_t>(fps * 2));
        tof->tofBaseNode.depth.link(benchmark->input);
        auto reportQueue = benchmark->report.createOutputQueue();

        pipeline.start();

        bool timedOut = false;
        reportQueue->get<dai::BenchmarkReport>(kReportTimeout, timedOut);  // Warm-up
        REQUIRE_FALSE(timedOut);
        for(int i = 0; i < 3; ++i) {
            auto report = reportQueue->get<dai::BenchmarkReport>(kReportTimeout, timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(report != nullptr);
            CHECK(report->fps == Catch::Approx(fps).margin(fps * 0.1f));
        }

        pipeline.stop();
    }
}
