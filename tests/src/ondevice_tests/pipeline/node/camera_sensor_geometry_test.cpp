#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("Camera ISP intrinsics follow sensor readout geometry", "[camera-sensor-geometry]") {
    const bool calibrateAtReadoutMode = GENERATE(false, true);
    CAPTURE(calibrateAtReadoutMode);
    dai::DeviceInfo deviceInfo;
    std::vector<dai::CameraFeatures> cameras;
    dai::CalibrationHandler calibration;
    dai::Platform platform;
    {
        dai::Device device;
        platform = device.getPlatform();
        if(platform != dai::Platform::RVC2 && platform != dai::Platform::RVC4) SKIP("Requires RVC2 or RVC4");
        deviceInfo = device.getDeviceInfo();
        deviceInfo.state = X_LINK_ANY_STATE;
        cameras = device.getConnectedCameraFeatures();
        calibration = device.readCalibration();
    }

    size_t testedModes = 0;
    size_t testedCroppedModes = 0;
    for(const auto& camera : cameras) {
        if(camera.width <= 0 || camera.height <= 0) continue;
        auto configs = camera.configs;
        const auto isSupported = [](const auto& config) { return config.type == dai::CameraSensorType::COLOR || config.type == dai::CameraSensorType::MONO; };
        const auto getFov = [&](const auto& config) {
            auto fov = config.fov;
            if(fov.isNormalized()) fov = fov.denormalize(camera.width, camera.height);
            return fov;
        };
        const auto isCropped = [&](const auto& config) {
            const auto fov = getFov(config);
            return isSupported(config) && fov.width > 0 && fov.height > 0
                   && (fov.x != 0 || fov.y != 0 || fov.width != camera.width || fov.height != camera.height);
        };
        // Prefer the reported native 4K case; other sensors exercise their own
        // cropped mode so the regression also covers mono and smaller cameras.
        auto cropped = std::find_if(
            configs.begin(), configs.end(), [&](const auto& config) { return config.width == 3840 && config.height == 2160 && isCropped(config); });
        if(cropped == configs.end()) cropped = std::find_if(configs.begin(), configs.end(), isCropped);
        auto reference = cropped;
        // Full-FOV-only sensors still exercise calibration in binned pixels
        // and full sensor pixels, or a full-resolution control if not binned.
        if(reference == configs.end()) {
            reference = std::find_if(configs.begin(), configs.end(), [&](const auto& config) {
                return isSupported(config) && config.width > 0 && config.height > 0 && config.width < camera.width && config.height < camera.height;
            });
        }
        if(reference == configs.end()) reference = std::find_if(configs.begin(), configs.end(), isSupported);
        if(reference == configs.end()) continue;
        const auto calibrationMode = *reference;
        const auto calibrationFov = getFov(calibrationMode);
        const auto isCalibrationMode = [&](const auto& config) { return config.width == calibrationMode.width && config.height == calibrationMode.height; };
        // Exercise the calibration mode first, followed by full-FOV and binning controls.
        std::stable_sort(configs.begin(), configs.end(), [&](const auto& a, const auto& b) { return isCalibrationMode(a) && !isCalibrationMode(b); });
        std::set<std::pair<std::int32_t, std::int32_t>> testedSizes;
        for(const auto& config : configs) {
            // Bound device memory use and avoid testing duplicate FPS/HDR modes.
            if(!isSupported(config)) continue;
            if(config.width <= 0 || config.height <= 0 || static_cast<std::int64_t>(config.width) * config.height > 16000000) continue;
            if(!testedSizes.emplace(config.width, config.height).second) continue;
            const auto fov = getFov(config);
            if(fov.width <= 0 || fov.height <= 0) continue;
            const bool fullFov = fov.x == 0 && fov.y == 0 && fov.width == camera.width && fov.height == camera.height;
            if(!isCalibrationMode(config) && !fullFov) continue;
            CAPTURE(camera.socket, camera.sensorName, config.width, config.height, fov.x, fov.y, fov.width, fov.height);

            // Noncentral principal points expose crop errors that a centered
            // calibration could hide. Calibration applies only to this pipeline.
            const std::vector<std::vector<float>> fullK = {
                {1000.0f, 0.0f, camera.width * 0.5f + 37.0f}, {0.0f, 1100.0f, camera.height * 0.5f - 23.0f}, {0.0f, 0.0f, 1.0f}};
            const std::vector<float> distortion = {0.01f, -0.02f, 0.001f, -0.001f, 0.005f, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            auto localCalibration = calibration;
            if(calibrateAtReadoutMode) {
                auto readoutK = fullK;
                readoutK[0][2] -= calibrationFov.x;
                readoutK[1][2] -= calibrationFov.y;
                for(size_t column = 0; column < 3; ++column) {
                    readoutK[0][column] *= calibrationMode.width / calibrationFov.width;
                    readoutK[1][column] *= calibrationMode.height / calibrationFov.height;
                }
                if(platform == dai::Platform::RVC4) {
                    // Native cropped calibration is explicit. Unlabelled RVC4
                    // records retain their historical full-FOV interpretation.
                    localCalibration.setCameraIntrinsicsFromSensorCrop(
                        camera.socket, readoutK, calibrationMode.width, calibrationMode.height, camera.width, camera.height, calibrationFov);
                } else {
                    // Preserve the existing RVC2 native-calibration convention.
                    localCalibration.setCameraIntrinsics(camera.socket, readoutK, calibrationMode.width, calibrationMode.height);
                }
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

            // Wait for the previous firmware session to reset. Keep the known
            // endpoint for PoE devices, whose rediscovery can outlast a USB reset.
            const auto resetStart = std::chrono::steady_clock::now();
            bool ready = false;
            while(std::chrono::steady_clock::now() - resetStart < std::chrono::seconds(30)) {
                const auto devices = dai::XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, true, 50);
                ready = std::any_of(devices.begin(), devices.end(), [&](const auto& info) {
                    return info.deviceId == deviceInfo.deviceId
                           && (info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER
                               || (platform == dai::Platform::RVC4 && info.state == X_LINK_GATE));
                });
                if(ready) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            REQUIRE(ready);
            auto device = std::make_shared<dai::Device>(deviceInfo);
            dai::Pipeline pipeline(device);
            pipeline.setCalibrationData(localCalibration);
            const float fps = std::clamp(10.0f, config.minFps, config.maxFps);
            auto cameraNode = pipeline.create<dai::node::Camera>()->build(
                camera.socket, std::make_pair(static_cast<std::uint32_t>(config.width), static_cast<std::uint32_t>(config.height)), fps);
            // Bound host bandwidth on slower RVC4 links while keeping the
            // requested sensor mode and sensor FPS unchanged.
            const auto streamFps = platform == dai::Platform::RVC4 ? std::make_optional(1.0f) : std::nullopt;
            // Exercise an explicit native-sized stream independently of the
            // ISP scaler chosen for the other requested outputs.
            auto nativeQueue = cameraNode
                                   ->requestOutput({static_cast<std::uint32_t>(config.width), static_cast<std::uint32_t>(config.height)},
                                                   std::nullopt,
                                                   dai::ImgResizeMode::STRETCH,
                                                   streamFps,
                                                   false)
                                   ->createOutputQueue(1, false);
            auto ispQueue = cameraNode->requestIspOutput(streamFps)->createOutputQueue(1, false);
            const bool testResize = config.width % 2 == 0 && config.height % 2 == 0;
            std::shared_ptr<dai::MessageQueue> resizedQueue;
            if(testResize) {
                resizedQueue = cameraNode
                                   ->requestOutput({static_cast<std::uint32_t>(config.width / 2), static_cast<std::uint32_t>(config.height / 2)},
                                                   std::nullopt,
                                                   dai::ImgResizeMode::STRETCH,
                                                   streamFps,
                                                   false)
                                   ->createOutputQueue(1, false);
            }
            pipeline.start();

            auto expectedFullK = fullK;
            bool calibrationUpdated = false;
            auto checkFrame = [&](const std::shared_ptr<dai::MessageQueue>& queue, std::optional<float> outputScale, const char* outputPath) {
                CAPTURE(outputPath);
                bool timedOut = false;
                auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
                REQUIRE_FALSE(timedOut);
                REQUIRE(frame != nullptr);
                const auto updateDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
                while(calibrationUpdated && std::fabs(frame->getTransformation().getSourceIntrinsicMatrix()[0][0] - fullK[0][0]) < 1.0f) {
                    REQUIRE(std::chrono::steady_clock::now() < updateDeadline);
                    frame = queue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
                    REQUIRE_FALSE(timedOut);
                    REQUIRE(frame != nullptr);
                }
                if(outputScale) {
                    REQUIRE(frame->getWidth() == static_cast<int>(config.width * *outputScale));
                    REQUIRE(frame->getHeight() == static_cast<int>(config.height * *outputScale));
                } else {
                    REQUIRE(frame->getWidth() > 0);
                    REQUIRE(frame->getHeight() > 0);
                    REQUIRE(static_cast<std::int64_t>(frame->getWidth()) * config.height == static_cast<std::int64_t>(frame->getHeight()) * config.width);
                }
                const auto transform = frame->getTransformation();
                const auto actualK = transform.getIntrinsicMatrix();
                auto expectedK = expectedFullK;
                expectedK[0][2] -= fov.x;
                expectedK[1][2] -= fov.y;
                // A crop translates the principal point; binning/scaling also
                // changes focal lengths. Output resizing composes afterwards.
                for(size_t column = 0; column < 3; ++column) {
                    expectedK[0][column] *= frame->getWidth() / fov.width;
                    expectedK[1][column] *= frame->getHeight() / fov.height;
                }
                CAPTURE(frame->getWidth(), frame->getHeight(), actualK, expectedK);
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
            checkFrame(ispQueue, platform == dai::Platform::RVC4 ? std::nullopt : std::make_optional(1.0f), "ISP");
            checkFrame(nativeQueue, 1.0f, "native-sized request");
            if(resizedQueue) checkFrame(resizedQueue, 0.5f, "half-sized request");
            if(platform == dai::Platform::RVC4 && !calibrateAtReadoutMode && isCalibrationMode(config) && isCropped(config)) {
                // Change physical calibration at runtime using an explicit native
                // capture ROI. Changed focal length distinguishes fresh metadata
                // from queued frames; checking all K entries then catches a
                // missing readout crop on the update path.
                expectedFullK[0][0] += 123;
                expectedFullK[1][1] += 77;
                expectedFullK[0][2] += 7;
                expectedFullK[1][2] -= 5;
                auto updatedK = expectedFullK;
                updatedK[0][2] -= calibrationFov.x;
                updatedK[1][2] -= calibrationFov.y;
                for(size_t column = 0; column < 3; ++column) {
                    updatedK[0][column] *= calibrationMode.width / calibrationFov.width;
                    updatedK[1][column] *= calibrationMode.height / calibrationFov.height;
                }
                auto updatedCalibration = localCalibration;
                updatedCalibration.setCameraIntrinsicsFromSensorCrop(
                    camera.socket, updatedK, calibrationMode.width, calibrationMode.height, camera.width, camera.height, calibrationFov);
                device->setCalibration(updatedCalibration);
                calibrationUpdated = true;
                checkFrame(ispQueue, std::nullopt, "ISP after runtime calibration");
                checkFrame(nativeQueue, 1.0f, "native-sized request after runtime calibration");
                if(resizedQueue) checkFrame(resizedQueue, 0.5f, "half-sized request after runtime calibration");
            }
            ++testedModes;
            if(fov.x != 0 || fov.y != 0 || fov.width != camera.width || fov.height != camera.height) ++testedCroppedModes;
        }
    }
    if(testedModes == 0) SKIP("No supported color or mono sensor mode");
    INFO("Checked " << testedModes << " sensor modes, including " << testedCroppedModes << " cropped modes");
    if(testedCroppedModes == 0) WARN("This device has no cropped sensor mode; only full-FOV controls were exercised");
}

TEST_CASE("RVC4 preserves calibration from a processed full-FOV readout", "[camera-resized-calibration-compatibility]") {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("RVC4 calibration compatibility regression");
    for(const auto& camera : device->getConnectedCameraFeatures()) {
        const auto physicalFov = [&](const dai::CameraSensorConfig& mode) {
            auto fov = mode.fov;
            if(fov.isNormalized()) fov = fov.denormalize(camera.width, camera.height);
            return fov;
        };
        const auto cropped = std::find_if(camera.configs.begin(), camera.configs.end(), [&](const auto& mode) {
            const auto fov = physicalFov(mode);
            // A 16:9 calibration output can also be produced by cropping a
            // full-FOV 4:3 readout, without selecting this native crop mode.
            return (mode.type == dai::CameraSensorType::COLOR || mode.type == dai::CameraSensorType::MONO) && mode.width == 1920 && mode.height == 1080
                   && fov.width > 0 && fov.height > 0 && fov.width < camera.width;
        });
        if(cropped == camera.configs.end()) continue;
        const auto full = std::find_if(camera.configs.begin(), camera.configs.end(), [&](const auto& mode) {
            const auto fov = physicalFov(mode);
            return mode.width > 0 && mode.height > 0 && static_cast<std::int64_t>(mode.width) * mode.height <= 16000000 && fov.x == 0 && fov.y == 0
                   && fov.width == camera.width && fov.height == camera.height;
        });
        if(full == camera.configs.end()) continue;
        CAPTURE(camera.socket, camera.sensorName, camera.width, camera.height);
        const std::vector<std::vector<float>> fullK = {{1000, 0, camera.width * .5f + 37}, {0, 1100, camera.height * .5f - 23}, {0, 0, 1}};
        const float scale = std::max(static_cast<float>(cropped->width) / camera.width, static_cast<float>(cropped->height) / camera.height);
        const float cropX = (camera.width - cropped->width / scale) * .5f;
        const float cropY = (camera.height - cropped->height / scale) * .5f;
        const auto nativeFov = physicalFov(*cropped);
        if(std::fabs(nativeFov.x - cropX) < .01f && std::fabs(nativeFov.y - cropY) < .01f) continue;

        // This K describes a normal CENTER_CROP Camera output from a full-FOV
        // readout. Matching its dimensions to a native mode loses provenance.
        auto outputCalibrationK = fullK;
        outputCalibrationK[0][2] -= cropX;
        outputCalibrationK[1][2] -= cropY;
        for(size_t column = 0; column < 3; ++column) {
            outputCalibrationK[0][column] *= scale;
            outputCalibrationK[1][column] *= scale;
        }
        auto calibration = device->readCalibration();
        calibration.setCameraIntrinsics(camera.socket, outputCalibrationK, cropped->width, cropped->height);
        calibration.setDistortionCoefficients(camera.socket, std::vector<float>(14, 0));
        calibration.setCameraType(camera.socket, dai::CameraModel::Perspective);
        dai::Pipeline pipeline(device);
        pipeline.setCalibrationData(calibration);
        const auto cameraNode =
            pipeline.create<dai::node::Camera>()->build(camera.socket,
                                                        std::make_pair(static_cast<std::uint32_t>(full->width), static_cast<std::uint32_t>(full->height)),
                                                        std::clamp(10.0f, full->minFps, full->maxFps));
        const auto queue =
            cameraNode
                ->requestOutput(
                    {static_cast<std::uint32_t>(full->width), static_cast<std::uint32_t>(full->height)}, std::nullopt, dai::ImgResizeMode::STRETCH, 1.0f, false)
                ->createOutputQueue(1, false);
        pipeline.start();
        bool timedOut = false;
        const auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getWidth() == full->width);
        REQUIRE(frame->getHeight() == full->height);
        auto expected = fullK;
        for(size_t column = 0; column < 3; ++column) {
            expected[0][column] *= static_cast<float>(full->width) / camera.width;
            expected[1][column] *= static_cast<float>(full->height) / camera.height;
        }
        const auto actual = frame->getTransformation().getIntrinsicMatrix();
        CAPTURE(actual, expected);
        for(size_t row = 0; row < 3; ++row)
            for(size_t column = 0; column < 3; ++column) CHECK(actual[row][column] == Catch::Approx(expected[row][column]).margin(.01f));
        return;
    }
    SKIP("Requires a cropped mode whose dimensions also describe a distinct resized full-FOV output");
}
