#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

TEST_CASE("OAK4-D firmware resolves IMU extrinsics without board-config defaults") {
    dai::Device::Config config;
    config.board.defaultImuExtr.clear();
    dai::Device device(config);
    const auto original = device.getCalibration();
    auto missing = original.getEepromData();
    const auto imuType = device.getConnectedIMU();
    if(missing.productName != "OAK4-D" || (imuType != "accel" && imuType != "ACCEL" && imuType != "LSM6" && imuType != "lsm6")) {
        SKIP("Requires an OAK4-D with the firmware's accel/LSM6 design extrinsics");
    }
    auto measured = missing;
    measured.imuExtrinsics = dai::Extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {1, 2, 3}, dai::CameraBoardSocket::CAM_A);
    measured.imuExtrinsics.specTranslation = {4, 5, 6};
    device.setCalibration(dai::CalibrationHandler(measured));
    const auto preserved = device.getCalibration();
    device.setCalibration(original);
    CHECK(preserved.getEepromData().imuExtrinsics.translation.x == 1);
    CHECK(preserved.getEepromData().imuExtrinsics.specTranslation.x == 4);

    // Change only volatile calibration. An identity rotation with no camera reference
    // reproduces the EEPROM contents which previously bypassed the FW design defaults.
    missing.imuExtrinsics = {};
    missing.imuExtrinsics.rotationMatrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    device.setCalibration(dai::CalibrationHandler(missing));
    const auto resolved = device.getCalibration();
    device.setCalibration(original);
    CHECK(resolved.getEepromData().imuExtrinsics.toCameraSocket != dai::CameraBoardSocket::AUTO);
    for(const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        REQUIRE_NOTHROW(resolved.getCameraToImuExtrinsics(socket, true, dai::LengthUnit::METER));
    }
}

TEST_CASE("Device VIO can stop before receiving any sensor input") {
    dai::Pipeline pipeline;
    REQUIRE(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4);
    const auto vio = pipeline.create<dai::node::VIO>();
    const auto queue = vio->transform.createOutputQueue(4, false);
    REQUIRE_FALSE(vio->runOnHost());
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(queue->tryGet() == nullptr);
    pipeline.stop();
    pipeline.wait();
}

// Requires a calibrated stereo/IMU OAK-4 looking at a well-lit, textured scene.
TEST_CASE("Device VIO returns finite poses with exact source timestamps and restarts") {
    for(int session = 0; session < 3; ++session) {
        dai::Pipeline pipeline;
        REQUIRE(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4);
        const auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30);
        const auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30);
        const auto imu = pipeline.create<dai::node::IMU>();
        const auto sync = pipeline.create<dai::node::Sync>();
        sync->setRunOnHost(false);
        sync->setTimestampSource(dai::node::Sync::TimestampSource::DEVICE);
        sync->setSyncThreshold(std::chrono::milliseconds(5));
        const auto vio = pipeline.create<dai::node::VIO>();
        REQUIRE_FALSE(vio->runOnHost());
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);
        const auto leftOutput = left->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, std::nullopt, false);
        leftOutput->link(sync->inputs["left"]);
        right->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, std::nullopt, false)->link(sync->inputs["right"]);
        sync->out.link(vio->stereo);
        // Session 1 delays IMU explicitly; session 2 exercises the normal device-only path.
        const auto delayedImu = session == 1 ? imu->out.createOutputQueue(100, false) : nullptr;
        const auto imuInput = session == 1 ? vio->imu.createInputQueue(64, true) : nullptr;
        if(session == 2) imu->out.link(vio->imu);
        // Only this regression test also exports images to verify pose metadata.
        const auto frames = leftOutput->createOutputQueue(32, false);
        const auto poses = vio->transform.createOutputQueue(32, false);
        std::map<int64_t, dai::Buffer> imageMetadata;
        std::vector<std::shared_ptr<dai::TransformData>> samples;
        pipeline.start();
        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + std::chrono::seconds(30);
        while(pipeline.isRunning() && std::chrono::steady_clock::now() < deadline) {
            if(delayedImu) {
                for(const auto& data : delayedImu->tryGetAll<dai::IMUData>()) {
                    // Discard early samples instead of replaying an old IMU backlog.
                    if(std::chrono::steady_clock::now() - start >= std::chrono::seconds(5)) {
                        REQUIRE(imuInput->trySend(data));
                    }
                }
            }
            for(const auto& frame : frames->tryGetAll<dai::ImgFrame>()) {
                REQUIRE(frame != nullptr);
                imageMetadata[frame->getSequenceNum()].setBufferMetadataFrom(frame);
            }
            for(const auto& pose : poses->tryGetAll<dai::TransformData>()) {
                REQUIRE(pose != nullptr);
                samples.push_back(pose);
            }
            if(session == 0 && imageMetadata.size() >= 100) break;
            if(samples.size() >= 20 && imageMetadata.count(samples.back()->getSequenceNum()) != 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if(session == 0) {
            // More than 64 startup frames must not exhaust VIO metadata while IMU is absent.
            REQUIRE(imageMetadata.size() >= 100);
            REQUIRE(samples.empty());
            pipeline.stop();
            pipeline.wait();
            continue;
        }
        REQUIRE(samples.size() >= 20);
        auto previousTime = std::chrono::steady_clock::time_point{};
        for(const auto& pose : samples) {
            const auto source = imageMetadata.find(pose->getSequenceNum());
            REQUIRE(source != imageMetadata.end());
            CHECK(pose->getTimestamp() == source->second.getTimestamp());
            CHECK(pose->getTimestampDevice() == source->second.getTimestampDevice());
            CHECK(pose->getTimestampSystem() == source->second.getTimestampSystem());
            CHECK(pose->getTimestampDevice() > previousTime);
            const auto position = pose->getTranslation();
            const auto rotation = pose->getQuaternion();
            for(const auto value : {position.x, position.y, position.z, rotation.qx, rotation.qy, rotation.qz, rotation.qw}) CHECK(std::isfinite(value));
            previousTime = pose->getTimestampDevice();
        }
        pipeline.stop();
        pipeline.wait();
    }
}
