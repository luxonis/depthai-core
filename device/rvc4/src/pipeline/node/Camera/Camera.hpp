#pragma once

#include <spdlog/async_logger.h>
#include <stdint.h>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "camera/CameraSensor.hpp"
#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/ImageManip/ImageManip.hpp"
#include "utilities/FpsRegulator.hpp"

namespace dai {
namespace gate {

class Camera : public node::Camera {
   public:
    constexpr static const char* NAME = node::Camera::NAME;

    float sourceHFovDegree = 0.0f;

    uint32_t calibrationWidth = 0;
    uint32_t calibrationHeight = 0;
    ImgTransformation baseTransformation;

    Camera() = delete;

    Camera(std::unique_ptr<Properties> props) : node::Camera(std::move(props)) {}

    bool runOnHost() const override {
        return true;
    }
    void run() override;
    void stop() override;
    void buildStage1() override;
    void buildStage2() override;
    void buildStage3() override;
    using SensorErrorStateCallback = CameraSensor::SensorErrorStateCallback;
    void* sensorErrorCbAdd(SensorErrorStateCallback cb);
    void sensorErrorCbRemove(void* opaqueHandle);
    void setToFExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso);
    void sendBuffer(CameraSensor::FrameCallbackArgs);
    void setDeviceId(const std::string& deviceId);
    std::shared_ptr<dai::ImgFrame> makeFrame(
        CameraSensor::FrameCallbackArgs args, std::optional<dai::Timestamp> tsSystem, dai::ImgFrame::Type type, bool interleaved, dai::CameraBoardSocket socket, ImgTransformation transformation);

   private:
    struct SensorConfig {
        int modeIndex;
        CameraSensorConfig mode;
        float fps;
    };

    class CameraOutput {
       public:
        size_t index{};
        Camera* camera{};
        std::unique_ptr<FpsRegulator> fpsRegulator;
        std::unique_ptr<ManipResizer> resizer;
        ImgTransformation transformation;
        size_t resolutionWidth{};
        size_t resolutionHeight{};
        uint32_t eepromId;
        bool ispOutput;
    };

    size_t findMaxResolutionIndex() const;

    std::vector<size_t> getIspIndices() const;

    size_t getMaxFpsIndex() const;

    static std::vector<dai::CameraBoardSocket> camerasInThePipeline;

    size_t maxResolutionIndex;
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    std::unique_ptr<CameraSensor> cameraSensor;
    std::mutex tofExposureMutex;
    bool tofExposureEnabled = true;
    bool tofExposureLocked = false;
    std::optional<uint32_t> tofExposureLimitUs;
    bool isToFSensor{false};
    int numRaw = 0;
    ImgFrame::Fsync camSyncType = dai::ImgFrame::Fsync::NONE;
    bool warnedForMissingCalibration = false;
    std::vector<CameraOutput> currentOutputs;
    std::atomic<bool> isGstPipelineRunning{false};
    bool mockIspEnabled = false;
    std::string deviceId;
    std::shared_ptr<const dai::beta::MultiDeviceCalibrationHandler> multiDeviceCalibration;
    uint32_t multiDeviceCalibrationId = 0;
    bool warnedForMultiDeviceMismatch = false;
    void refreshMultiDeviceCalibration();
    std::optional<SensorConfig> sensorConfig;
    dai::CameraSensorType selectedSensorType = dai::CameraSensorType::AUTO;

    void setupResizers(const SensorConfig& sensorConfig);
    std::shared_ptr<ImgFrame> toCorrectFormat(ImgFrameCapability cap, std::shared_ptr<ImgFrame> inFrame);
    void runGstPipelineInThread();
    float getFps();
    SensorConfig returnSensorConfig(const std::pair<int, dai::CameraSensorConfig>& mode, float requestedFps) const;
    SensorConfig decideSensorConfig();
    std::vector<std::pair<int, dai::CameraSensorConfig>> filterByHDR(const std::vector<std::pair<int, dai::CameraSensorConfig>>& modes) const;
    static std::vector<std::pair<int, dai::CameraSensorConfig>> filterByFps(const std::vector<std::pair<int, dai::CameraSensorConfig>>& modes,
                                                                            float requestedFps);
    std::vector<std::tuple<uint32_t, uint32_t, std::optional<float>>> getOutputsScalerSizes() const;
    std::pair<uint32_t, uint32_t> getLowestSatisfyingSensorSize() const;
    void validateSettings();
    void setDefaultExposureMaster();
};

}  // namespace gate
}  // namespace dai
