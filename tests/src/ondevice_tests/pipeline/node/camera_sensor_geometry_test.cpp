#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("Camera ISP intrinsics follow native 4K sensor geometry", "[camera-sensor-geometry]") {
    const bool calibrateInSelectedMode = GENERATE(false, true);
    CAPTURE(calibrateInSelectedMode);
    dai::DeviceInfo deviceInfo;
    std::vector<dai::CameraFeatures> cameras;
    dai::CalibrationHandler calibration;
    {
        dai::Device device;
        if(device.getPlatform() != dai::Platform::RVC2) {
            SKIP("This regression exercises the RVC2 ISP calibration path");
        }
        deviceInfo.deviceId = device.getDeviceId();
        deviceInfo.platform = device.getDeviceInfo().platform;
        cameras = device.getConnectedCameraFeatures();
        calibration = device.readCalibration();
    }

    size_t testedModes = 0;
    size_t testedCroppedModes = 0;
    for(const auto& camera : cameras) {
        if(camera.width <= 0 || camera.height <= 0) continue;
        auto configs = camera.configs;
        const auto isNative4k = [](const auto& config) { return config.width == 3840 && config.height == 2160; };
        if(std::none_of(configs.begin(), configs.end(), isNative4k)) continue;
        // Exercise the reported native 4K path first, followed by full-FOV controls.
        std::stable_sort(configs.begin(), configs.end(), [&](const auto& a, const auto& b) { return isNative4k(a) && !isNative4k(b); });
        std::set<std::pair<std::int32_t, std::int32_t>> testedSizes;
        for(const auto& config : configs) {
            // Bound device memory use and avoid testing duplicate FPS/HDR modes.
            if(config.type != dai::CameraSensorType::COLOR) continue;
            if(config.width <= 0 || config.height <= 0 || static_cast<std::int64_t>(config.width) * config.height > 16000000) continue;
            if(!testedSizes.emplace(config.width, config.height).second) continue;
            auto fov = config.fov;
            if(fov.isNormalized()) fov = fov.denormalize(camera.width, camera.height);
            if(fov.width <= 0 || fov.height <= 0) continue;
            const bool fullFov = fov.x == 0 && fov.y == 0 && fov.width == camera.width && fov.height == camera.height;
            if(!isNative4k(config) && !fullFov) continue;
            CAPTURE(camera.socket, camera.sensorName, config.width, config.height, fov.x, fov.y, fov.width, fov.height);

            // Noncentral principal points expose crop errors that a centered
            // calibration could hide. Calibration applies only to this pipeline.
            const std::vector<std::vector<float>> fullK = {
                {1000.0f, 0.0f, camera.width * 0.5f + 37.0f}, {0.0f, 1100.0f, camera.height * 0.5f - 23.0f}, {0.0f, 0.0f, 1.0f}};
            const std::vector<float> distortion = {0.01f, -0.02f, 0.001f, -0.001f, 0.005f, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            auto localCalibration = calibration;
            if(calibrateInSelectedMode) {
                auto readoutK = fullK;
                readoutK[0][2] -= fov.x;
                readoutK[1][2] -= fov.y;
                for(size_t column = 0; column < 3; ++column) {
                    readoutK[0][column] *= config.width / fov.width;
                    readoutK[1][column] *= config.height / fov.height;
                }
                localCalibration.setCameraIntrinsics(camera.socket, readoutK, config.width, config.height);
            } else {
                localCalibration.setCameraIntrinsics(camera.socket, fullK, camera.width, camera.height);
            }
            localCalibration.setDistortionCoefficients(camera.socket, distortion);
            localCalibration.setCameraType(camera.socket, dai::CameraModel::Perspective);

            std::optional<std::vector<std::vector<float>>> expectedPose;
            try {
                expectedPose = localCalibration.getCameraExtrinsics(camera.socket, localCalibration.getCameraWithLowestId(), false);
            } catch(const std::runtime_error&) {
                // Some camera modules have intrinsics without cross-camera calibration.
            }

            // Allow the previous USB connection to disappear before reusing the
            // device, as in camera_fps_config_test. Opening during reset can
            // otherwise attach to the previous firmware session.
            const auto resetStart = std::chrono::steady_clock::now();
            while(std::chrono::steady_clock::now() - resetStart < std::chrono::seconds(2)) {
                const auto devices = dai::XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, true, 50);
                if(std::none_of(devices.begin(), devices.end(), [&](const auto& info) { return info.deviceId == deviceInfo.deviceId; })) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            auto device = std::make_shared<dai::Device>(deviceInfo);
            dai::Pipeline pipeline(device);
            pipeline.setCalibrationData(localCalibration);
            const float fps = std::clamp(10.0f, config.minFps, config.maxFps);
            auto cameraNode = pipeline.create<dai::node::Camera>()->build(
                camera.socket, std::make_pair(static_cast<std::uint32_t>(config.width), static_cast<std::uint32_t>(config.height)), fps);
            auto ispQueue = cameraNode->requestIspOutput()->createOutputQueue(1, false);
            const bool testResize = config.width % 2 == 0 && config.height % 2 == 0;
            std::shared_ptr<dai::MessageQueue> resizedQueue;
            if(testResize) {
                resizedQueue = cameraNode
                                   ->requestOutput({static_cast<std::uint32_t>(config.width / 2), static_cast<std::uint32_t>(config.height / 2)},
                                                   std::nullopt,
                                                   dai::ImgResizeMode::STRETCH,
                                                   std::nullopt,
                                                   false)
                                   ->createOutputQueue(1, false);
            }
            pipeline.start();

            auto checkFrame = [&](const std::shared_ptr<dai::MessageQueue>& queue, float outputScale) {
                bool timedOut = false;
                auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
                REQUIRE_FALSE(timedOut);
                REQUIRE(frame != nullptr);
                REQUIRE(frame->getWidth() == static_cast<int>(config.width * outputScale));
                REQUIRE(frame->getHeight() == static_cast<int>(config.height * outputScale));
                const auto transform = frame->getTransformation();
                const auto actualK = transform.getIntrinsicMatrix();
                auto expectedK = fullK;
                expectedK[0][2] -= fov.x;
                expectedK[1][2] -= fov.y;
                // A crop translates the principal point; binning/scaling also
                // changes focal lengths. Output resizing composes afterwards.
                for(size_t column = 0; column < 3; ++column) {
                    expectedK[0][column] *= config.width / fov.width * outputScale;
                    expectedK[1][column] *= config.height / fov.height * outputScale;
                }
                CAPTURE(outputScale, actualK, expectedK);
                for(size_t row = 0; row < 3; ++row) {
                    for(size_t column = 0; column < 3; ++column) {
                        CHECK(actualK[row][column] == Catch::Approx(expectedK[row][column]).margin(0.01f));
                    }
                }
                CHECK(transform.getDistortionModel() == dai::CameraModel::Perspective);
                CHECK(transform.getDistortionCoefficients() == distortion);
                if(expectedPose) {
                    const auto actualPose = transform.getExtrinsics().getTransformationMatrix();
                    for(size_t row = 0; row < 4; ++row) {
                        for(size_t column = 0; column < 4; ++column) {
                            CHECK(actualPose[row][column] == Catch::Approx((*expectedPose)[row][column]).margin(0.001f));
                        }
                    }
                }
            };
            checkFrame(ispQueue, 1.0f);
            if(resizedQueue) checkFrame(resizedQueue, 0.5f);
            ++testedModes;
            if(fov.x != 0 || fov.y != 0 || fov.width != camera.width || fov.height != camera.height) ++testedCroppedModes;
        }
    }
    if(testedModes == 0) SKIP("This regression requires a color camera with a native 3840x2160 sensor mode");
    INFO("Checked " << testedModes << " sensor modes, including " << testedCroppedModes << " cropped modes");
    if(testedCroppedModes == 0) WARN("This device has no cropped sensor mode; only full-FOV controls were exercised");
}
