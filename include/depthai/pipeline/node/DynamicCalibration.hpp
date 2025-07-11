#pragma once

#include <CalibrationHandle.hpp>
#include <DynamicCalibration.hpp>
#include <SensorHandle.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
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

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "DynamicCalibration";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    ~DynamicCalibration() override = default;

    struct CalibrationStateMachine {
        enum class CalibrationState { Idle, InitializingPipeline, LoadingImages, ProcessingQuality, Recalibrating, ResetDynamicRecalibration };

        enum class CalibrationMode { None, QualityCheck, Recalibration };

        CalibrationState state = CalibrationState::Idle;
        CalibrationMode mode = CalibrationMode::None;
        bool pipelineReady = false;

        void startQualityCheck();

        void startRecalibration();

        void markPipelineReady();

        bool isIdle() const;

        void AdvanceAfterLoading();

        void deleteAllData();

        void finish();

        std::string stateToString() const;
    };

    /**
     * Input DynamicCalibrationConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{
        *this,
        {"inputConfig", DEFAULT_GROUP, NON_BLOCKING_QUEUE, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::DynamicCalibrationConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";
    std::string rightInputName = "right";

    void buildInternal() override;

    /**
     * Input left image
     */
    Input& left = inputs[leftInputName];

    /**
     * Input right image
     */
    Input& right = inputs[rightInputName];

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
     * Set Dynamic recalibration performance mode
     */
    void setPerformanceMode(dcl::PerformanceMode mode);

    /**
     * Set Dynamic recalibration as Continious mode, no user interaction needed
     */
    void setContinousMode();

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
    void setInternalCalibration(
        std::shared_ptr<Device> device, const CameraBoardSocket socketSrc, const CameraBoardSocket socketDest, const int width, const int height);
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

    dai::DynamicCalibrationResults::CalibrationQualityResult calibQualityfromDCL(const dcl::CalibrationQuality& src);
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
    CalibData getDataFromDAIHandler(
        CalibrationHandler currentCalibration, const CameraBoardSocket boardSocketA, const CameraBoardSocket boardSocketB, const int width, const int height);

    /**
     * Calibration state machine, which holds the state of Node and provide stabile enviroment;
     * - Initialization of pipeline,
     * - Loading images in DCL,
     * - Starting Calibration Check,
     * - Starting of Recalibration
     * - Reseting of data
     */
    dcl::CalibrationQuality calibQuality;
    CalibrationStateMachine calibrationSM;

    void resetResults();

    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 1, {{DatatypeEnum::MessageGroup, true}}}};
};

}  // namespace node
}  // namespace dai
