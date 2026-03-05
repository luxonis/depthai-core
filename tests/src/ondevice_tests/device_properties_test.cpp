#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {
class TempFile {
   public:
    TempFile(const std::string& prefix, const std::vector<std::uint8_t>& data) {
        auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::temp_directory_path() / (prefix + "_" + std::to_string(now) + ".bin");
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    }

    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    TempFile(TempFile&&) = delete;
    TempFile& operator=(TempFile&&) = delete;

    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }

    const std::filesystem::path& getPath() const {
        return path;
    }

   private:
    std::filesystem::path path;
};

TempFile writeTempFile(const std::string& prefix, const std::vector<std::uint8_t>& data) {
    return TempFile(prefix, data);
}

std::uintmax_t getFileSize(const TempFile& file) {
    return std::filesystem::file_size(file.getPath());
}
}  // namespace

TEST_CASE("Test pipeline device property setters apply before start") {
    dai::Pipeline pipeline;

    pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();

    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);

    const int32_t xlinkSize = 64 * 1024;
    const uint32_t sippSize = dai::DeviceProperties::SIPP_BUFFER_DEFAULT_SIZE + 1024;
    const uint32_t sippDmaSize = dai::DeviceProperties::SIPP_DMA_BUFFER_DEFAULT_SIZE + 1024;

    auto tuningPath = writeTempFile("dai_cam_tuning", {1, 2, 3, 4, 5, 6});
    auto tuningSocketPath = writeTempFile("dai_cam_tuning_cam_a", {7, 8, 9});

    pipeline.setXLinkChunkSize(xlinkSize);
    pipeline.setSippBufferSize(static_cast<int>(sippSize));
    pipeline.setSippDmaBufferSize(static_cast<int>(sippDmaSize));
    pipeline.setCameraTuningBlobPath(tuningPath.getPath());
    pipeline.setCameraTuningBlobPath(dai::CameraBoardSocket::CAM_A, tuningSocketPath.getPath());

    auto props = device->getProperties();
    REQUIRE(props.xlinkChunkSize == xlinkSize);
    REQUIRE(props.sippBufferSize == sippSize);
    REQUIRE(props.sippDmaBufferSize == sippDmaSize);

    REQUIRE(props.cameraTuningBlobSize.has_value());
    REQUIRE(props.cameraTuningBlobSize.value() == getFileSize(tuningPath));
    REQUIRE(props.cameraTuningBlobUri == "asset:camTuning");

    REQUIRE(props.cameraSocketTuningBlobSize.count(dai::CameraBoardSocket::CAM_A) == 1);
    REQUIRE(props.cameraSocketTuningBlobUri.count(dai::CameraBoardSocket::CAM_A) == 1);
    REQUIRE(props.cameraSocketTuningBlobSize.at(dai::CameraBoardSocket::CAM_A) == getFileSize(tuningSocketPath));
    REQUIRE(props.cameraSocketTuningBlobUri.at(dai::CameraBoardSocket::CAM_A)
            == "asset:camTuning_" + std::to_string(static_cast<int>(dai::CameraBoardSocket::CAM_A)));

    pipeline.build();
    pipeline.start();

    pipeline.setXLinkChunkSize(xlinkSize + 1024);
    pipeline.setSippBufferSize(static_cast<int>(sippSize + 1024));
    pipeline.setSippDmaBufferSize(static_cast<int>(sippDmaSize + 1024));

    auto propsAfterStart = device->getProperties();
    REQUIRE(propsAfterStart.xlinkChunkSize == xlinkSize);
    REQUIRE(propsAfterStart.sippBufferSize == sippSize);
    REQUIRE(propsAfterStart.sippDmaBufferSize == sippDmaSize);

    pipeline.stop();
}

TEST_CASE("Test device setProperties before and after start") {
    dai::Pipeline pipeline;
    pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();

    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);

    auto props = device->getProperties();

    const int32_t xlinkSize = (props.xlinkChunkSize == -1) ? 32 * 1024 : props.xlinkChunkSize + 1024;
    const uint32_t sippSize = props.sippBufferSize + 2048;
    const uint32_t sippDmaSize = props.sippDmaBufferSize + 2048;

    props.xlinkChunkSize = xlinkSize;
    props.sippBufferSize = sippSize;
    props.sippDmaBufferSize = sippDmaSize;

    device->setProperties(props);

    auto propsAfterSet = device->getProperties();
    REQUIRE(propsAfterSet.xlinkChunkSize == xlinkSize);
    REQUIRE(propsAfterSet.sippBufferSize == sippSize);
    REQUIRE(propsAfterSet.sippDmaBufferSize == sippDmaSize);

    pipeline.build();
    pipeline.start();

    props.xlinkChunkSize = xlinkSize + 2048;
    props.sippBufferSize = sippSize + 2048;
    props.sippDmaBufferSize = sippDmaSize + 2048;
    device->setProperties(props);

    auto propsAfterStart = device->getProperties();
    REQUIRE(propsAfterStart.xlinkChunkSize == xlinkSize);
    REQUIRE(propsAfterStart.sippBufferSize == sippSize);
    REQUIRE(propsAfterStart.sippDmaBufferSize == sippDmaSize);

    pipeline.stop();
}

TEST_CASE("Test pipeline device property setters keep device and host properties in sync") {
    dai::Pipeline pipeline;
    pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();

    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    REQUIRE(pipeline.getDefaultDeviceProperties()->eepromId == 1);

    auto checkSync = [&](const std::string& stage) {
        auto deviceProps = device->getProperties();
        auto pipelinePropsOpt = pipeline.getDefaultDeviceProperties();
        INFO(stage);
        REQUIRE(pipelinePropsOpt.has_value());

        const auto& pipelineProps = pipelinePropsOpt.value();
        REQUIRE(deviceProps.xlinkChunkSize == pipelineProps.xlinkChunkSize);
        REQUIRE(deviceProps.sippBufferSize == pipelineProps.sippBufferSize);
        REQUIRE(deviceProps.sippDmaBufferSize == pipelineProps.sippDmaBufferSize);

        REQUIRE(deviceProps.cameraTuningBlobSize.has_value() == pipelineProps.cameraTuningBlobSize.has_value());
        if(deviceProps.cameraTuningBlobSize.has_value()) {
            REQUIRE(deviceProps.cameraTuningBlobSize.value() == pipelineProps.cameraTuningBlobSize.value());
            REQUIRE(deviceProps.cameraTuningBlobUri == pipelineProps.cameraTuningBlobUri);
        }

        REQUIRE(deviceProps.cameraSocketTuningBlobSize.count(dai::CameraBoardSocket::CAM_A)
                == pipelineProps.cameraSocketTuningBlobSize.count(dai::CameraBoardSocket::CAM_A));
        if(deviceProps.cameraSocketTuningBlobSize.count(dai::CameraBoardSocket::CAM_A) == 1) {
            REQUIRE(deviceProps.cameraSocketTuningBlobSize.at(dai::CameraBoardSocket::CAM_A)
                    == pipelineProps.cameraSocketTuningBlobSize.at(dai::CameraBoardSocket::CAM_A));
            REQUIRE(deviceProps.cameraSocketTuningBlobUri.at(dai::CameraBoardSocket::CAM_A)
                    == pipelineProps.cameraSocketTuningBlobUri.at(dai::CameraBoardSocket::CAM_A));
        }

        REQUIRE(deviceProps.calibData.has_value() == pipelineProps.calibData.has_value());
        if(deviceProps.calibData.has_value()) {
            REQUIRE(deviceProps.calibData->deviceName == pipelineProps.calibData->deviceName);
        }
        REQUIRE(deviceProps.eepromId == pipelineProps.eepromId);
    };

    const int32_t xlinkSize = 48 * 1024;
    const uint32_t sippSize = dai::DeviceProperties::SIPP_BUFFER_DEFAULT_SIZE + 512;
    const uint32_t sippDmaSize = dai::DeviceProperties::SIPP_DMA_BUFFER_DEFAULT_SIZE + 512;

    auto tuningPath = writeTempFile("dai_cam_tuning_sync", {1, 3, 5, 7});
    auto tuningSocketPath = writeTempFile("dai_cam_tuning_sync_cam_a", {2, 4, 6});

    pipeline.setXLinkChunkSize(xlinkSize);
    pipeline.setSippBufferSize(static_cast<int>(sippSize));
    pipeline.setSippDmaBufferSize(static_cast<int>(sippDmaSize));
    pipeline.setCameraTuningBlobPath(tuningPath.getPath());
    pipeline.setCameraTuningBlobPath(dai::CameraBoardSocket::CAM_A, tuningSocketPath.getPath());
    checkSync("after pipeline setters");

    dai::DeviceProperties props = device->getProperties();
    props.xlinkChunkSize = xlinkSize + 1024;
    props.sippBufferSize = sippSize + 1024;
    props.sippDmaBufferSize = sippDmaSize + 1024;
    pipeline.setDefaultDeviceProperties(props);
    checkSync("after setDefaultDeviceProperties");

    dai::DeviceProperties externalProps = device->getProperties();
    pipeline.setDefaultDevicePropertiesRef(&externalProps);
    pipeline.setXLinkChunkSize(xlinkSize + 2048);
    pipeline.setSippBufferSize(static_cast<int>(sippSize + 2048));
    pipeline.setSippDmaBufferSize(static_cast<int>(sippDmaSize + 2048));
    checkSync("after setDefaultDevicePropertiesRef + setters");

    auto calibHandler = device->readCalibration();
    calibHandler.setDeviceName("sync_device_name");
    pipeline.setCalibrationData(calibHandler);
    checkSync("after setCalibrationData");

    auto eepromData = device->getCalibration().getEepromData();
    eepromData.deviceName = "sync_device_name_2";
    pipeline.setEepromData(eepromData);
    checkSync("after setEepromData");

    pipeline.build();
    pipeline.start();
    checkSync("after start");

    auto eepromDataPostStart = device->getCalibration().getEepromData();
    eepromDataPostStart.deviceName = "sync_device_name_post_start";
    pipeline.setEepromData(eepromDataPostStart);
    checkSync("after setEepromData post start");

    pipeline.stop();
}
