#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

void checkRate(const std::vector<std::chrono::steady_clock::time_point>& timestamps, double expected, const char* stream) {
    INFO("stream=" << stream << ", requested=" << expected);
    REQUIRE(timestamps.size() >= 2);
    REQUIRE(timestamps.front() > std::chrono::steady_clock::time_point{});
    for(std::size_t i = 1; i < timestamps.size(); ++i) REQUIRE(timestamps[i] > timestamps[i - 1]);
    const auto elapsed = std::chrono::duration<double>(timestamps.back() - timestamps.front()).count();
    CHECK(static_cast<double>(timestamps.size() - 1) / elapsed == Catch::Approx(expected).epsilon(0.02));
    // Count intervals over elapsed time: fixed buckets alone have >2% edge error at 5Hz.
    REQUIRE(timestamps.back() - timestamps.front() >= std::chrono::seconds(2));
    for(auto begin = timestamps.begin(); begin != timestamps.end(); ++begin) {
        const auto end = std::lower_bound(begin + 1, timestamps.end(), *begin + std::chrono::seconds(2));
        if(end == timestamps.end()) break;
        const auto window = std::chrono::duration<double>(*end - *begin).count();
        CHECK(static_cast<double>(end - begin) / window == Catch::Approx(expected).epsilon(0.02));
    }
}

void testImuAlignment(int fps, dai::IMUSensor accelerometer, dai::IMUSensor magnetometer) {
    CAPTURE(fps, accelerometer, magnetometer);
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only IMU regression");
    const auto features = device->getConnectedCameraFeatures();
    if(std::none_of(features.begin(), features.end(), [](const auto& feature) { return feature.socket == dai::CameraBoardSocket::CAM_B; })) {
        SKIP("Requires CAM_B for the RVC4 camera/IMU fixture");
    }
    if(device->getConnectedIMU().empty() || device->getConnectedIMU() == "BMI270") SKIP("Requires accelerometer and magnetometer");

    const auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, static_cast<float>(fps));
    camera->initialControl.setManualExposure(1000, 100);
    auto* output = camera->requestOutput({320, 200}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, static_cast<float>(fps));
    const auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(accelerometer, 200);
    imu->enableIMUSensor(magnetometer, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);
    const auto sync = pipeline.create<dai::node::Sync>();
    sync->setTimestampSource(dai::node::Sync::TimestampSource::DEVICE);
    sync->setSyncThreshold(std::chrono::milliseconds(10));
    sync->setSyncAttempts(-1);
    sync->setSyncOnIndividualReports(true);
    sync->setRunOnHost(false);
    output->link(sync->inputs["image"]);
    imu->out.link(sync->inputs["imu"]);
    const auto imageQueue = output->createOutputQueue(4000, false);
    const auto imuQueue = imu->out.createOutputQueue(4000, false);
    const auto syncQueue = sync->out.createOutputQueue(4000, false);

    std::vector<std::shared_ptr<dai::ImgFrame>> images;
    std::vector<std::shared_ptr<dai::IMUData>> reports;
    std::vector<std::shared_ptr<dai::MessageGroup>> groups;
    pipeline.start();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    bool started = false;
    while(std::chrono::steady_clock::now() < deadline) {
        const auto newImages = imageQueue->tryGetAll<dai::ImgFrame>();
        images.insert(images.end(), newImages.begin(), newImages.end());
        const auto newReports = imuQueue->tryGetAll<dai::IMUData>();
        reports.insert(reports.end(), newReports.begin(), newReports.end());
        const auto newGroups = syncQueue->tryGetAll<dai::MessageGroup>();
        groups.insert(groups.end(), newGroups.begin(), newGroups.end());
        if(!started && !images.empty() && !reports.empty()) {
            started = true;
            // Three seconds of warmup, 15 seconds measured, and a tail for Sync.
            deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(18500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    pipeline.stop();
    REQUIRE(started);
    REQUIRE(images.front() != nullptr);
    REQUIRE(reports.front() != nullptr);
    const auto lower =
        std::max(images.front()->getTimestampDevice(dai::CameraExposureOffset::MIDDLE), reports.front()->getTimestampDevice()) + std::chrono::seconds(3);
    const auto upper = lower + std::chrono::seconds(15);
    const auto tolerance = std::chrono::milliseconds(10) + std::chrono::nanoseconds(1);
    std::vector<std::chrono::steady_clock::time_point> imageTimes, headerTimes, accelTimes, magTimes;
    std::vector<int64_t> imageSequences;
    for(const auto& image : images) {
        REQUIRE(image != nullptr);
        CHECK(image->getWidth() == 320);
        CHECK(image->getHeight() == 200);
        CHECK(image->getType() == dai::ImgFrame::Type::GRAY8);
        const auto timestamp = image->getTimestampDevice(dai::CameraExposureOffset::MIDDLE);
        CHECK(timestamp <= image->getTimestampDevice(dai::CameraExposureOffset::END));
        if(timestamp < lower || timestamp > upper) continue;
        if(!imageSequences.empty()) REQUIRE(image->getSequenceNum() == imageSequences.back() + 1);
        imageSequences.push_back(image->getSequenceNum());
        imageTimes.push_back(timestamp);
    }
    for(const auto& report : reports) {
        REQUIRE(report != nullptr);
        REQUIRE(report->packets.size() == 1);
        const auto& packet = report->packets.front();
        CHECK(std::isfinite(packet.acceleroMeter.x));
        CHECK(std::isfinite(packet.acceleroMeter.y));
        CHECK(std::isfinite(packet.acceleroMeter.z));
        CHECK(std::isfinite(packet.magneticField.x));
        CHECK(std::isfinite(packet.magneticField.y));
        CHECK(std::isfinite(packet.magneticField.z));
        const auto header = report->getTimestampDevice();
        if(header < lower || header > upper) continue;
        const auto accel = packet.acceleroMeter.getTimestampDevice();
        const auto mag = packet.magneticField.getTimestampDevice();
        CHECK(std::chrono::abs(header - std::max(accel, mag)) <= std::chrono::nanoseconds(1));
        CHECK(std::chrono::abs(accel - mag) <= tolerance);
        headerTimes.push_back(header);
        accelTimes.push_back(accel);
        magTimes.push_back(mag);
    }
    checkRate(imageTimes, fps, "image");
    // Paired output is limited by the 100Hz magnetometer, even with 200Hz accel.
    checkRate(headerTimes, 100, "IMU packet");
    checkRate(accelTimes, 100, "accelerometer");
    checkRate(magTimes, 100, "magnetometer");
    CHECK(imageTimes.front() <= lower + std::chrono::duration<double>(2.0 / fps));
    CHECK(imageTimes.back() >= upper - std::chrono::duration<double>(2.0 / fps));
    CHECK(headerTimes.front() <= lower + std::chrono::milliseconds(20));
    CHECK(headerTimes.back() >= upper - std::chrono::milliseconds(20));

    std::vector<std::chrono::steady_clock::time_point> syncedImages, syncedAccel, syncedMag;
    std::optional<int64_t> previousSyncedSequence;
    for(const auto& group : groups) {
        REQUIRE(group != nullptr);
        const auto image = group->get<dai::ImgFrame>("image");
        const auto report = group->get<dai::IMUData>("imu");
        REQUIRE(image != nullptr);
        REQUIRE(report != nullptr);
        REQUIRE(report->packets.size() == 1);
        CHECK(image->getWidth() == 320);
        CHECK(image->getHeight() == 200);
        CHECK(image->getType() == dai::ImgFrame::Type::GRAY8);
        const auto timestamp = image->getTimestampDevice(dai::CameraExposureOffset::MIDDLE);
        CHECK(timestamp <= image->getTimestampDevice(dai::CameraExposureOffset::END));
        const auto& packet = report->packets.front();
        CHECK(std::isfinite(packet.acceleroMeter.x));
        CHECK(std::isfinite(packet.acceleroMeter.y));
        CHECK(std::isfinite(packet.acceleroMeter.z));
        CHECK(std::isfinite(packet.magneticField.x));
        CHECK(std::isfinite(packet.magneticField.y));
        CHECK(std::isfinite(packet.magneticField.z));
        if(!std::binary_search(imageSequences.begin(), imageSequences.end(), image->getSequenceNum())) continue;
        if(previousSyncedSequence) CHECK(image->getSequenceNum() > *previousSyncedSequence);
        previousSyncedSequence = image->getSequenceNum();
        const auto accel = packet.acceleroMeter.getTimestampDevice();
        const auto mag = packet.magneticField.getTimestampDevice();
        CHECK(std::max({timestamp, accel, mag}) - std::min({timestamp, accel, mag}) <= tolerance);
        syncedImages.push_back(timestamp);
        syncedAccel.push_back(accel);
        syncedMag.push_back(mag);
    }
    CHECK(static_cast<double>(syncedImages.size()) / imageTimes.size() >= 0.98);
    checkRate(syncedImages, fps, "synced image");
    checkRate(syncedAccel, fps, "synced accelerometer");
    checkRate(syncedMag, fps, "synced magnetometer");
}

}  // namespace

TEST_CASE("RVC4 raw IMU rates and image alignment", "[imu][alignment][rvc4]") {
    const auto fps = GENERATE(5, 10, 20, 30, 40, 45, 50);
    testImuAlignment(fps, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::MAGNETOMETER_RAW);
}

TEST_CASE("RVC4 uncalibrated IMU rates and image alignment", "[imu][alignment][rvc4]") {
    const auto fps = GENERATE(5, 10, 20, 30, 40, 45, 50);
    testImuAlignment(fps, dai::IMUSensor::ACCELEROMETER_UNCALIBRATED, dai::IMUSensor::MAGNETOMETER_UNCALIBRATED);
}

TEST_CASE("RVC4 calibrated IMU rates and image alignment", "[imu][alignment][rvc4]") {
    const auto fps = GENERATE(5, 10, 20, 30, 40, 45, 50);
    testImuAlignment(fps, dai::IMUSensor::ACCELEROMETER_CALIBRATED, dai::IMUSensor::MAGNETOMETER_CALIBRATED);
}

TEST_CASE("RVC4 IMU non-native requested rates", "[imu][rate][rvc4]") {
    const auto fps = GENERATE(40, 45);
    CAPTURE(fps);
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only IMU regression");
    if(device->getConnectedIMU().empty() || device->getConnectedIMU() == "BMI270") SKIP("Requires accelerometer and magnetometer");
    const auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, fps);
    imu->enableIMUSensor(dai::IMUSensor::MAGNETOMETER_RAW, fps);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);
    const auto queue = imu->out.createOutputQueue(4000, false);
    std::vector<std::chrono::steady_clock::time_point> headers, accel, mag;
    std::optional<std::chrono::steady_clock::time_point> lower;
    pipeline.start();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while(std::chrono::steady_clock::now() < deadline) {
        for(const auto& report : queue->tryGetAll<dai::IMUData>()) {
            REQUIRE(report != nullptr);
            REQUIRE(report->packets.size() == 1);
            const auto header = report->getTimestampDevice();
            if(!lower) {
                lower = header + std::chrono::seconds(3);
                deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(18500);
            }
            if(header < *lower || header > *lower + std::chrono::seconds(15)) continue;
            const auto& packet = report->packets.front();
            headers.push_back(header);
            accel.push_back(packet.acceleroMeter.getTimestampDevice());
            mag.push_back(packet.magneticField.getTimestampDevice());
            CHECK(std::chrono::abs(header - std::max(accel.back(), mag.back())) <= std::chrono::nanoseconds(1));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    pipeline.stop();
    REQUIRE(lower.has_value());
    checkRate(headers, fps, "IMU packet");
    checkRate(accel, fps, "accelerometer");
    checkRate(mag, fps, "magnetometer");
    CHECK(headers.front() <= *lower + std::chrono::duration<double>(2.0 / fps));
    CHECK(headers.back() >= *lower + std::chrono::seconds(15) - std::chrono::duration<double>(2.0 / fps));
}
