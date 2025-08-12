#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>

#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
namespace dai {
namespace node {

struct DclUtils {
    static void convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationA,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationB,
                                           const CameraBoardSocket socketSrc,
                                           const CameraBoardSocket socketDest,
                                           const int width,
                                           const int height);

    static std::shared_ptr<dcl::CameraCalibrationHandle> createDclCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                              const std::vector<float> distortionCoefficients,
                                                                              const std::vector<std::vector<float>> rotationMatrix,
                                                                              const std::vector<float> translationVector);

    static std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> convertDaiCalibrationToDcl(
        const CalibrationHandler& currentCalibration,
        const CameraBoardSocket boardSocketA,
        const CameraBoardSocket boardSocketB,
        const int width,
        const int height);

    static dcl::ImageData cvMatToImageData(const cv::Mat& mat);
};

/**
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DynamicCalibration";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    // clang-format off
    // Constructors with dcl::DynamicCalibration ... mainly for testing purposes
    DynamicCalibration(std::unique_ptr<dcl::DynamicCalibration> dc)
      : DeviceNodeCRTP()
      , dynCalibImpl(std::move(dc)) {}
    DynamicCalibration(const std::shared_ptr<Device>& device, std::unique_ptr<dcl::DynamicCalibration> dc)
      : DeviceNodeCRTP(device)
      , dynCalibImpl(std::move(dc)) {}
    DynamicCalibration(std::unique_ptr<Properties> props, bool confMode, std::unique_ptr<dcl::DynamicCalibration> dc)
      : DeviceNodeCRTP(std::move(props), confMode)
      , dynCalibImpl(std::move(dc)) {}
    DynamicCalibration(
        const std::shared_ptr<Device>& device,
        std::unique_ptr<Properties> props,
        bool confMode,
        std::unique_ptr<dcl::DynamicCalibration> dc)
      : DeviceNodeCRTP(device, std::move(props), confMode)
      , dynCalibImpl(std::move(dc)) {}
    // clang-format on

    ~DynamicCalibration() override = default;

    enum ErrorCode : int {
        OK = 0,
        QUALITY_CHECK_FAILED = 1,
        CALIBRATION_FAILED = 2,
        PIPELINE_INITIALIZATION_FAILED = 4,
        EMPTY_IMAGE_QUEUE = 5,
        MISSING_IMAGE = 6,
        CALIBRATION_DOES_NOT_EXIST = 7,
        STOP_LOADING_IMAGES_DURING_RECALIBRATION = 8,
    };

    // clang-format off
    /**
     * Input DynamicCalibrationConfig message with ability to modify parameters in runtime.
     */
    Input commandInput{
        *this,
        {
	  "inputConfig",
	  DEFAULT_GROUP,
	  NON_BLOCKING_QUEUE,
	  1,  // Queue_size -> only one command at the time
	  {{{DatatypeEnum::DynamicCalibrationCommand, false}}},
	  DEFAULT_WAIT_FOR_MESSAGE
	}
    };

    Input configInput{*this, {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, 1, {{{DatatypeEnum::DynamicCalibrationConfig, true}}}, false}};

    /**
     * Output calibration quality result
     */
    Output calibrationOutput{
        *this,
	{
	    "calibrationOutput",
	    DEFAULT_GROUP,
	    {{{DatatypeEnum::DynamicCalibrationResult, false}}}
	}
    };

    Output qualityOutput{
        *this,
	{
	    "qualityOutput",
	    DEFAULT_GROUP,
	    {{{DatatypeEnum::CalibrationQuality, false}}}
	}
    };

    Output coverageOutput{
        *this,
	{
	    "coverageResult",
	    DEFAULT_GROUP,
	    {{{DatatypeEnum::CoverageData, false}}}
	}
    };

    Input syncInput{
        *this,
	{
	    "inSync",
	    DEFAULT_GROUP,
	    false,
	    1,
	    {{DatatypeEnum::MessageGroup, true}}
	}
    };
    // clang-format on

    void buildInternal() override;

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;
    std::string leftInputName = "left";
    std::string rightInputName = "right";
    /**
     * Input left image
     */
    Input& left = inputs[leftInputName];

    /**
     * Input right image
     */
    Input& right = inputs[rightInputName];

    void setInitialConfig(DynamicCalibrationConfig& config) {
        properties.initialConfig = config;
    }
    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    
    int getWidth() const {
        return width;
    }

    int getHeight() const {
        return height;
    }

    void setWidth(const int newWidth) {
        width = newWidth;
    }

    void setHeight(const int newHeight) {
        height = newHeight;
    }

    CameraBoardSocket getBorderSockerA() {
        return daiSocketA;
    }

    CameraBoardSocket getBorderSockerB() {
        return daiSocketB;
    }

    ErrorCode runQualityCheck(const bool force = false);

    ErrorCode runCalibration(const dai::CalibrationHandler& calibHandler, const bool force = false);

    ErrorCode runLoadImage(const bool blocking = false);

    ErrorCode computeCoverage();

    ErrorCode initializePipeline(const std::shared_ptr<dai::Device> daiDevice);

    // clang-format off
    ErrorCode doWorkContinuous(
        std::chrono::steady_clock::time_point& previousCalibrationTime,
	std::chrono::steady_clock::time_point& previousLoadingTime);
    // clang-format on

    ErrorCode doWork(std::chrono::steady_clock::time_point& previousLoadingTime);

    ErrorCode evaluateCommand(const std::shared_ptr<DynamicCalibrationCommand> command);

    void run() override;

   protected:
    Properties& getProperties() override;

   private:
    /**
     * From dai::CalibrationHandler data convert to DCL dcl::CameraCalibrationHandle, which includes all necesarry data for recalibration
     * @return dcl::CameraCalibrationHanlder
     */
    /**
     * Overwrites the internal calibration of DCL with new Calibration data provided by node.
     */
    // clang-format off
    void setCalibration(CalibrationHandler& handler);
    // clang-format on

    /**
     * From  DCL dcl::CameraCalibrationHandle convert to dai::CalibrationHandler, so device can setCalibration
     * @return dai::CalibrationHandlerr
     */
    dai::CalibrationQuality calibQualityfromDCL(const dcl::CalibrationQuality& src);
    /**
     * DCL held properties
     */
    std::shared_ptr<dcl::CameraSensorHandle> sensorA;
    std::shared_ptr<dcl::CameraSensorHandle> sensorB;
    std::shared_ptr<dcl::Device> dcDevice;
    dcl::mxid_t deviceName;
    dcl::socket_t socketA;
    dcl::socket_t socketB;

    /**
     * DAI held properties
     */
    CalibrationHandler calibrationHandler;

    CameraBoardSocket daiSocketA = CameraBoardSocket::CAM_B;
    CameraBoardSocket daiSocketB = CameraBoardSocket::CAM_C;
    int width;
    int height;

    // std::chrono::milliseconds sleepingTime = 250ms;
    // static constexpr std::chrono::milliseconds kSleepingTime{250};
    std::chrono::milliseconds sleepingTime{250};

    bool recalibrationRunning = false;
    bool slept = false;

    /**
     * Calibration state machine, which holds the state of Node and provide stabile enviroment;
     * - Initialization of pipeline,
     * - Loading images in DCL,
     * - Starting Calibration Check,
     * - Starting of Recalibration
     * - Reseting of data
     */
    bool runOnHostVar = true;
    const std::unique_ptr<dcl::DynamicCalibration> dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
};

}  // namespace node
}  // namespace dai
