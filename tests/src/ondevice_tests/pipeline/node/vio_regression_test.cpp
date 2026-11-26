#include <catch2/catch_all.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/depthai.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"

// Regression tests for the BasaltVIO and RTABMapVIO fixes.
// The device cases need an RVC2 or RVC4 with an IMU and two mono cameras (CAM_B, CAM_C).
// They carry the `vio` label and no `ci` label, so CI does not run them. Run them by hand: ctest -L vio

namespace {

using Clock = std::chrono::steady_clock;

// Covers device boot and the first odometry frames on RVC2 and RVC4.
constexpr auto kOdometryTimeout = std::chrono::seconds(20);
constexpr size_t kMinOdometryMessages = 5;
constexpr int kMonoWidth = 640;
constexpr int kMonoHeight = 400;

// Waits until kMinOdometryMessages odometry messages arrive or the timeout passes.
std::vector<std::shared_ptr<dai::Odometry>> collectOdometry(dai::MessageQueue& queue) {
    std::vector<std::shared_ptr<dai::Odometry>> odometry;
    const auto deadline = Clock::now() + kOdometryTimeout;
    while(odometry.size() < kMinOdometryMessages) {
        const auto remaining = deadline - Clock::now();
        if(remaining <= Clock::duration::zero()) break;
        bool timedOut = false;
        auto msg = queue.get<dai::Odometry>(remaining, timedOut);
        if(timedOut) break;
        if(msg) odometry.push_back(msg);
    }
    return odometry;
}

void configureImu(dai::node::IMU& imu) {
    imu.enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu.setBatchReportThreshold(1);
    imu.setMaxBatchReports(10);
}

std::shared_ptr<dai::node::Camera> buildMono(dai::Pipeline& pipeline, dai::CameraBoardSocket socket) {
    return pipeline.create<dai::node::Camera>()->build(socket, std::nullopt, 30);
}

void requireStampedInWindow(const dai::Odometry& msg, Clock::time_point startedAt) {
    REQUIRE(msg.getTimestamp() >= startedAt);
    REQUIRE(msg.getTimestamp() <= Clock::now());
}

}  // namespace

TEST_CASE("BasaltVIO: stop() before the pipeline starts does not crash", "[vio][basalt]") {
    // Before the fix, stop() pushed to null queues when initialize() had not run, which crashed the pipeline on shutdown.
    // stop() is private on BasaltVIO; the pipeline reaches it through the ThreadedNode interface, so the test does too.
    dai::Pipeline pipeline;
    auto vio = pipeline.create<dai::node::BasaltVIO>();
    dai::ThreadedNode& node = *vio;
    REQUIRE_NOTHROW(node.stop());
}

TEST_CASE("BasaltVIO: odometry is stamped with the time of its frame", "[vio][basalt][device]") {
    dai::Pipeline pipeline;
    auto left = buildMono(pipeline, dai::CameraBoardSocket::CAM_B);
    auto right = buildMono(pipeline, dai::CameraBoardSocket::CAM_C);
    auto imu = pipeline.create<dai::node::IMU>();
    auto vio = pipeline.create<dai::node::BasaltVIO>();
    configureImu(*imu);
    left->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(vio->left);
    right->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(vio->right);
    imu->out.link(vio->imu);
    auto odometryQueue = vio->transform.createOutputQueue();

    const auto startedAt = Clock::now();
    pipeline.start();
    auto odometry = collectOdometry(*odometryQueue);
    REQUIRE(odometry.size() >= kMinOdometryMessages);

    Clock::time_point previous = startedAt;
    for(const auto& msg : odometry) {
        requireStampedInWindow(*msg, startedAt);
        REQUIRE(msg->getTimestamp() >= previous);
        previous = msg->getTimestamp();
    }
}

TEST_CASE("BasaltVIO: IMU samples that go back in time do not abort", "[vio][basalt][device]") {
    // RVC2 firmware could stamp a BNO08x sample earlier than the previous one; Basalt asserts on that and aborts the process.
    dai::Pipeline pipeline;
    auto left = buildMono(pipeline, dai::CameraBoardSocket::CAM_B);
    auto right = buildMono(pipeline, dai::CameraBoardSocket::CAM_C);
    auto vio = pipeline.create<dai::node::BasaltVIO>();
    left->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(vio->left);
    right->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(vio->right);
    auto imuQueue = vio->imu.createInputQueue();
    auto odometryQueue = vio->transform.createOutputQueue();

    pipeline.start();
    std::vector<std::shared_ptr<dai::Odometry>> odometry;
    auto stamp = Clock::now();
    const auto deadline = stamp + kOdometryTimeout;
    // Feed long enough for Basalt to integrate many out-of-order samples, not only until the first odometry.
    const auto feedUntil = stamp + std::chrono::seconds(5);
    for(int i = 0; Clock::now() < deadline && (Clock::now() < feedUntil || odometry.size() < kMinOdometryMessages); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // Every 20th sample is stamped 1 ms before the previous one.
        stamp = i % 20 == 19 ? stamp - std::chrono::milliseconds(1) : Clock::now();
        dai::IMUPacket packet;
        packet.acceleroMeter.z = 9.81f;
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
        packet.acceleroMeter.timestamp = {ns / 1000000000, ns % 1000000000};
        packet.gyroscope.timestamp = packet.acceleroMeter.timestamp;
        auto imuData = std::make_shared<dai::IMUData>();
        imuData->packets.push_back(packet);
        imuQueue->send(imuData);
        while(auto msg = odometryQueue->tryGet<dai::Odometry>()) odometry.push_back(msg);
    }
    REQUIRE(odometry.size() >= kMinOdometryMessages);
}

TEST_CASE("RTABMapVIO: runs without a features input and stamps odometry", "[vio][rtabmap][device]") {
    dai::Pipeline pipeline;
    auto left = buildMono(pipeline, dai::CameraBoardSocket::CAM_B);
    auto right = buildMono(pipeline, dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto vio = pipeline.create<dai::node::RTABMapVIO>();
    configureImu(*imu);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(true);
    stereo->enableDistortionCorrection(true);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);
    left->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(stereo->left);
    right->requestOutput(std::make_pair(kMonoWidth, kMonoHeight))->link(stereo->right);
    stereo->rectifiedLeft.link(vio->rect);
    stereo->depth.link(vio->depth);
    imu->out.link(vio->imu);
    // The features input stays unlinked: start() must succeed with it unlinked.
    auto odometryQueue = vio->transform.createOutputQueue();

    const auto startedAt = Clock::now();
    REQUIRE_NOTHROW(pipeline.start());
    auto odometry = collectOdometry(*odometryQueue);
    REQUIRE(odometry.size() >= kMinOdometryMessages);

    Clock::time_point previousDevice{};
    for(const auto& msg : odometry) {
        requireStampedInWindow(*msg, startedAt);
        REQUIRE(msg->getTimestampDevice() != Clock::time_point{});
        REQUIRE(msg->getTimestampDevice() >= previousDevice);
        previousDevice = msg->getTimestampDevice();
    }
}
