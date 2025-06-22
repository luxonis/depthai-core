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

    /**
     * Set Dynamic recalibration performance mode based on user input
     */
    void setPerformanceMode(dai::DynamicCalibrationConfig::AlgorithmControl::PerformanceMode mode);

    /**
     * Set Dynamic recalibration as Continious mode, no user interaction needed
     */
    void setContiniousMode();

    /**
     * Set time frequency, when new recalibration will be performed in Continious mode
     */
    void setTimeFrequency(int time);

    private:

    void run() override;
    /**
     * From dai::CalibrationHandler data convert to DCL dcl::CameraCalibrationHandle, which includes all necesarry data for recalibration
     * @return dcl::CameraCalibrationHanlder
     */
    std::shared_ptr<dcl::CameraCalibrationHandle> createDCLCameraCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                             const std::vector<float> distortionCoefficients,
                                                                             const std::vector<std::vector<float>> rotationMatrix,
                                                                             const std::vector<float> translationVector);
    /**
     * Set initial pipeline of DCL
     */
    void pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket boardSocketA, CameraBoardSocket boardSocketB, int width = 1280, int height = 800);

    /**
     * Overwrites the internal calibration of DCL with new Calibration data provided by node.
     */
    void setInternalCalibration(std::shared_ptr<Device> device,
                                const CameraBoardSocket socketSrc,
                                const CameraBoardSocket socketDest,
                                const int width,
                                const int height);
     /**
     * From  DCL dcl::CameraCalibrationHandle convert to dai::CalibrationHandler, so device can setCalibration
     * @return dai::CalibrationHandlerr
     */                               
    CalibrationHandler convertDCLtoDAI(CalibrationHandler calibHandler,
                                       const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibration,
                                       const CameraBoardSocket socketSrc,
                                       const CameraBoardSocket socketDest,
                                       const int width,
                                       const int height);
     /**
     * DCL held properties
     */       
    std::shared_ptr<DynamicCalibrationConfig> calibrationConfig;
    std::unique_ptr<dcl::DynamicCalibration> dynCalibImpl;
    std::shared_ptr<dcl::CameraSensorHandle> sensorA;
    std::shared_ptr<dcl::CameraSensorHandle> sensorB;
    std::shared_ptr<dcl::Device> dcDevice;
    DynamicCalibrationResults dynResult;
    dcl::mxid_t deviceName;
    dcl::socket_t socketA;
    dcl::socket_t socketB;

    /**
     * DAI held properties
    */ 
    CameraBoardSocket daiSocketA;
    CameraBoardSocket daiSocketB;
    int widthDefault;
    int heightDefault;
    bool forceTrigger = false;


    struct CalibData {
        std::vector<float> translationVectorA;
        std::vector<std::vector<float>> rotationMatrixA;
        std::vector<float> translationVectorB;
        std::vector<std::vector<float>> rotationMatrixB;
        std::vector<std::vector<float>> leftCameraMatrix;
        std::vector<std::vector<float>> rightCameraMatrix;
        std::vector<float> leftDistortionCoefficients;
        std::vector<float> rightDistortionCoefficients;
    };

     /**
     * From  dai::CalibrationHandler to all necesarry information which needs to be provided to DCL
     * @return dcl::CameraCalibrationHanlder
     */  
    CalibData getDataFromDAIHandler(CalibrationHandler currentCalibration,
                                    const CameraBoardSocket boardSocketA,
                                    const CameraBoardSocket boardSocketB,
                                    const int width,
                                    const int height);

     /**
     * Calibration state machine, which holds the state of Node and provide stabile enviroment;  
     * - Initialization of pipeline, 
     * - Loading images in DCL, 
     * - Starting Calibration Check, 
     * - Starting of Recalibration
     * - Reseting of data
     */ 
    struct CalibrationStateMachine {
        enum class CalibrationState { Idle, InitializingPipeline, LoadingImages, ProcessingQuality, Recalibrating, ResetDynamicRecalibration };

        enum class CalibrationMode { None, QualityCheck, Recalibration };

        CalibrationState state = CalibrationState::Idle;
        CalibrationMode mode = CalibrationMode::None;
        bool pipelineReady = false;

        void startQualityCheck() {
            if(isIdle()) {
                state = CalibrationState::LoadingImages;
                mode = CalibrationMode::QualityCheck;
            }
        }

        void startRecalibration() {
            if(isIdle()) {
                state = CalibrationState::LoadingImages;
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

        void AdvanceAfterLoading() {
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
                case CalibrationState::LoadingImages:
                    return "LoadingImages";
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
