#pragma once

#include <CalibrationHandle.hpp>
#include <DynamicCalibration.hpp>
#include <SensorHandle.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>

#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"

namespace dai {
namespace node {

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
    Properties& getProperties() override;

   public:

    std::shared_ptr<DynamicCalibrationConfig> initialConfig = std::make_shared<DynamicCalibrationConfig>();

    /**
     * Input DynamicCalibrationConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{
        *this,
        {"inputConfig", DEFAULT_GROUP, NON_BLOCKING_QUEUE, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::DynamicCalibrationConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for left ImgFrame of left-right pair
     */
    Input left{*this, {"left", DEFAULT_GROUP, NON_BLOCKING_QUEUE, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame of left-right pair
     */
    Input right{*this, {"right", DEFAULT_GROUP, NON_BLOCKING_QUEUE, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Output calibration quality result
     */
    Output outputCalibrationResults{*this, {"outputCalibrationResults", DEFAULT_GROUP, {{{DatatypeEnum::DynamicCalibrationResults, false}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void setNewCalibration(CalibrationHandler);

    private:

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
    void startCalibQualityCheck();
    void startRecalibration();

    void run() override;

    void pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket boardSocketA, CameraBoardSocket boardSocketB, int width = 1280, int height = 800);
    void setInternalCalibration(std::shared_ptr<Device> device,
                                const CameraBoardSocket socketSrc,
                                const CameraBoardSocket socketDest,
                                const int width,
                                const int height);
    CalibrationHandler convertDCLtoDAI(CalibrationHandler calibHandler,
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
    bool forceTrigger = false;

    DynamicCalibrationResults dynResult;

    struct CalibrationStateMachine {
        enum class CalibrationState { Idle, InitializingPipeline, CollectingFeatures, ProcessingQuality, Recalibrating, ResetDynamicRecalibration };

        enum class CalibrationMode { None, QualityCheck, Recalibration };

        CalibrationState state = CalibrationState::Idle;
        CalibrationMode mode = CalibrationMode::None;
        bool pipelineReady = false;

        void startQualityCheck() {
            if(isIdle()) {
                state = CalibrationState::CollectingFeatures;
                mode = CalibrationMode::QualityCheck;
            }
        }

        void startRecalibration() {
            if(isIdle()) {
                state = CalibrationState::CollectingFeatures;
                mode = CalibrationMode::Recalibration;
            }
        }

        void markPipelineReady() {
            pipelineReady = true;
            state = CalibrationState::Idle;
        }

        bool isIdle() const {
            return state == CalibrationState::Idle;
        }

        void maybeAdvanceAfterCollection() {  // TODO, REPLACE WITH ANYTHING MEANINGFUL FROM THE DCL ITSELF
            if(mode == CalibrationMode::QualityCheck) {
                state = CalibrationState::ProcessingQuality;
            } else if(mode == CalibrationMode::Recalibration) {
                state = CalibrationState::Recalibrating;
            }
        }
        
        void deleteAllData() {
            state = CalibrationState::ResetDynamicRecalibration;
        }

        void finish() {
            state = CalibrationState::Idle;
            mode = CalibrationMode::None;
        }

        std::string stateToString() const {
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
                case CalibrationState::ResetDynamicRecalibration:
                    return "ResetingCalibration";
                default:
                    return "Unknown";
            }
        }
    };

    CalibrationStateMachine calibrationSM;
};

}  // namespace node
}  // namespace dai
