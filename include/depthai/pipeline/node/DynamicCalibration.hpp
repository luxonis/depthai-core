#pragma once

#include <CalibrationHandle.hpp>
#include <DynamicCalibration.hpp>
#include <SensorHandle.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include <DynamicCalibration.hpp>
#include <CalibrationHandle.hpp>
#include <SensorHandle.hpp>

namespace dai {
namespace node {

enum class CalibrationState { Idle, InitializingPipeline, CollectingFeatures, ProcessingQuality, Recalibrating };

enum class CalibrationMode { None, QualityCheck, Recalibration };

inline const char* toString(CalibrationState state) {
    switch(state) {
        case CalibrationState::Idle:
            return "Idle";
        case CalibrationState::InitializingPipeline:
            return "InitializingPipeline";
        case CalibrationState::CollectingFeatures:
            return "CollectingFeatures";
        case CalibrationState::ProcessingQuality:
            return "ProcessingQuality";
        case CalibrationState::Recalibrating:
            return "Recalibrating";
        default:
            return "Unknown";
    }
}

struct CalibrationResult {
    std::optional<CalibrationHandler> calibration;
    bool valid = false;
    std::string info;

    static CalibrationResult Invalid(std::string reason = "No result") {
        return CalibrationResult{std::nullopt, false, std::move(reason)};
    }
};

struct QualityResult {
    double value = -1.0f;
    bool valid = false;
    std::string info;

    static QualityResult Invalid(std::string reason = "No result") {
        return QualityResult{-1.0f, false, std::move(reason)};
    }
};

struct CalibrationResults {
    QualityResult quality;
    CalibrationResult calibration;

    void reset() {
        quality = QualityResult::Invalid();
        calibration = CalibrationResult::Invalid();
    }
};

struct CalibrationStateMachine {
    CalibrationState state = CalibrationState::Idle;
    CalibrationMode mode = CalibrationMode::None;
    int collectedFrames = 0;
    bool pipelineReady = false;

    void startQualityCheck() {
        if(isIdle()) {
            state = CalibrationState::InitializingPipeline;
            mode = CalibrationMode::QualityCheck;
            collectedFrames = 0;
            pipelineReady = false;
        }
    }

    void startRecalibration() {
        if(isIdle()) {
            state = CalibrationState::InitializingPipeline;
            mode = CalibrationMode::Recalibration;
            collectedFrames = 0;
            pipelineReady = false;
        }
    }

    void markPipelineReady() {
        pipelineReady = true;
        state = CalibrationState::CollectingFeatures;
    }

    bool isIdle() const {
        return state == CalibrationState::Idle;
    }

    void maybeAdvanceAfterCollection() {  // TODO, REPLACE WITH ANYTHING MEANINGFUL FROM THE DCL ITSELF
        if(mode == CalibrationMode::QualityCheck && collectedFrames >= 1) { 
            state = CalibrationState::ProcessingQuality;
        } else if(mode == CalibrationMode::Recalibration && collectedFrames >= 3) {
            state = CalibrationState::Recalibrating;
        }
    }

    void finish() {
        state = CalibrationState::Idle;
        mode = CalibrationMode::None;
        collectedFrames = 0;
        pipelineReady = false;
    }

    std::string stateToString() const {
        return std::string(toString(state));
    }
};

/**
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   private:
    bool runOnHostVar = true;

   public:
    constexpr static const char* NAME = "DynamicCalibration";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    virtual ~DynamicCalibration(); 


   protected:
    Properties& getProperties();

   public:
    CalibrationResults results;

    std::shared_ptr<DynamicCalibrationConfig> initialConfig = std::make_shared<DynamicCalibrationConfig>();

    // We could have a map here to perform the calibration on an arbitrary number of inputs for example: (left, right, rgb).
    // /**
    //  * A map of inputs
    //  */
    // InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input StereoDepthConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{
        *this, {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::DynamicCalibrationConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for left ImgFrame of left-right pair
     */
    Input left{*this, {"left", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame of left-right pair
     */
    Input right{*this, {"right", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Get the calibration quality (epipolar error)
     * @return Epipolar error in pixels
     */
    /**
     * Set calibration to the dai.Device
     */

    std::shared_ptr<dcl::CameraCalibrationHandle> createDCLCameraCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                             const std::vector<float> distortionCoefficients,
                                                                             const std::vector<std::vector<float>> rotationMatrix,
                                                                             const std::vector<float> translationVector);
    void setNewCalibration(CalibrationHandler);
    void startCalibQualityCheck();
    void startRecalibration();
    QualityResult getCalibQuality() const;
    CalibrationResult getNewCalibration() const;
    void setContinuousMode(dai::DynamicCalibrationConfig::AlgorithmControl::RecalibrationMode mode);

    void run() override;

   private:
    void pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket boardSocketA, CameraBoardSocket boardSocketB, int width = 1280, int height = 800);
    void setInternalCalibration(std::shared_ptr<Device> device,
                        const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationA,
                        const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationB,
                        const CameraBoardSocket socketSrc,
                        const CameraBoardSocket socketDest,
                        const int width,
                        const int height);
    CalibrationHandler convertDCLtoDAI(CalibrationHandler calibHandler ,
                                        const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibration,
                                        const CameraBoardSocket socketSrc,
                                        const CameraBoardSocket socketDest,
                                        const int width,
                                        const int height);

    std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
    std::unique_ptr<dcl::DynamicCalibration> dynCalibImpl;
    std::shared_ptr<dcl::Device> dcDevice;
    dcl::mxid_t deviceName;
    std::shared_ptr<dcl::CameraSensorHandle> sensorA;
    std::shared_ptr<dcl::CameraSensorHandle> sensorB;
    dcl::socket_t socketA;
    dcl::socket_t socketB;
    int widthDefault;
    int heightDefault;

    CalibrationStateMachine calibrationSM;
};

}  // namespace node
}  // namespace dai
