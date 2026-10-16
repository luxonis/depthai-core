#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <deque>
#include <initializer_list>
#include <memory>
#include <optional>
#include <set>
#include <thread>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/depthai.hpp"

TEST_CASE("Basalt odometry preserves stereo measurement timestamps") {
    INFO("Keep the calibrated stereo rig stationary and pointed at a textured scene during this test");
    using Clock = std::chrono::steady_clock;
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    const auto cameras = device->getConnectedCameraFeatures();
    for(const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        if(std::none_of(cameras.begin(), cameras.end(), [socket](const auto& camera) { return camera.socket == socket; })) {
            SKIP("Basalt timestamp regression requires calibrated stereo cameras on CAM_B/CAM_C");
        }
    }
    if(device->getConnectedIMU().empty() || device->getConnectedIMU() == "NONE") {
        SKIP("Basalt timestamp regression requires an IMU");
    }

    const auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30);
    const auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30);
    const auto imu = pipeline.create<dai::node::IMU>();
    const auto odometry = pipeline.create<dai::node::BasaltVIO>();
    left->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8)->link(odometry->left);
    right->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8)->link(odometry->right);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu->out.link(odometry->imu);
    odometry->setImuUpdateRate(200);
    odometry->runSyncOnHost(true);

    // Observe the actual synchronized input groups, not the latest-image passthrough.
    const auto inputQueue = odometry->sync->out.createOutputQueue(128, false);
    const auto poseQueue = odometry->transform.createOutputQueue(128, false);
    std::set<Clock::time_point> inputStamps;
    std::deque<Clock::time_point> pendingStamps;
    auto previousStamp = Clock::time_point{};
    constexpr std::size_t REQUIRED_POSES = 10;
    std::size_t matchedPoses = 0;

    pipeline.start();
    const auto deadline = Clock::now() + std::chrono::seconds(30);
    while(Clock::now() < deadline && matchedPoses < REQUIRED_POSES) {
        while(const auto group = inputQueue->tryGet<dai::MessageGroup>()) {
            for(const auto& entry : *group) {
                const auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(entry.second);
                REQUIRE(frame != nullptr);
                inputStamps.insert(frame->getTimestamp());
            }
        }
        while(matchedPoses + pendingStamps.size() < REQUIRED_POSES) {
            const auto pose = poseQueue->tryGet<dai::Odometry>();
            if(!pose) break;
            const auto stamp = pose->getTimestamp();
            REQUIRE(stamp > Clock::time_point{});
            REQUIRE(stamp > previousStamp);
            previousStamp = stamp;
            pendingStamps.push_back(stamp);
        }
        // Input and output queues can be delivered in either order. Allow input
        // observation to catch up without pairing messages by arrival order.
        while(!pendingStamps.empty() && inputStamps.count(pendingStamps.front()) != 0) {
            pendingStamps.pop_front();
            ++matchedPoses;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    pipeline.stop();
    REQUIRE(matchedPoses == REQUIRED_POSES);
    REQUIRE(pendingStamps.empty());
}
