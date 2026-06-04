#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <optional>

#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/utility/spimpl.h"

namespace spdlog {
class async_logger;
}
namespace dcl {
class CameraSensorHandle;
}
namespace dai {
namespace node {

/*u
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DynamicCalibration";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~DynamicCalibration() override = default;

    // clang-format off
    /**
     * Input DynamicCalibrationControl message with ability to modify parameters in runtime.
     */
    Input inputControl{
        *this,
        {
	  "inputControl",
	  DEFAULT_GROUP,
	  NON_BLOCKING_QUEUE,
	  5,  // Queue_size -> only one command at the time
	  {{{DatatypeEnum::DynamicCalibrationControl, false}}},
	  DEFAULT_WAIT_FOR_MESSAGE
	}
    };

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

    Output metricsOutput{
        *this,
	{
	    "metricsOutput",
	    DEFAULT_GROUP,
	    {{{DatatypeEnum::CalibrationMetrics, false}}}
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
    std::string rgbInputName = "rgb";
    std::vector<std::string> names;
    /**
     * Input left image
     */
    Input& left = inputs[leftInputName];

    /**
     * Input right image
     */
    Input& right = inputs[rightInputName];

    /**
     * Input rgb image
     */
    Input& rgb = inputs[rgbInputName];

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */

    void setRunOnHost(bool runOnHost);

    bool runOnHost() const override;

    void postBuildStage() override;

   protected:
    Properties& getProperties() override;

   private:
    struct ConnectedSensor {
        CameraBoardSocket socket;
        std::vector<std::vector<float>> intrinsics;
        std::vector<float> distortion;
        CameraModel distortionModel = CameraModel::Perspective;
        size_t connectionOrder;
        std::pair<int, int> resolution;
        std::shared_ptr<dcl::CameraSensorHandle> sensorDcl;
    };

    std::vector<ConnectedSensor> connectedSensors;
    std::vector<CameraBoardSocket> socketsInHandler;
    std::vector<std::vector<std::vector<float>>> socketToSensorExtrinsics;

    class Impl;
    spimpl::impl_ptr<Impl> pimplDCL;
    enum ErrorCode : int {
        OK = 0,
        QUALITY_CHECK_FAILED = 1,
        CALIBRATION_FAILED = 2,
        PIPELINE_INITIALIZATION_FAILED = 4,
        EMPTY_IMAGE_QUEUE = 5,
        MISSING_IMAGE = 6,
        CALIBRATION_DOES_NOT_EXIST = 7,
        STOP_LOADING_IMAGES_DURING_CALIBRATION = 8,
        INVALID_COMMAND = 9,
    };

    /**
     * Check if the node is set to run on host
     */

    void run() override;

    CameraBoardSocket getBorderSockerLeft() {
        return daiSocketLeft;
    }

    CameraBoardSocket getBorderSockerRight() {
        return daiSocketRight;
    }

    ErrorCode runQualityCheck(const bool force = false);

    ErrorCode runCalibration(const dai::CalibrationHandler& calibHandler, const bool force = false, const bool keepCameraCenters = true);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    ErrorCode runLoadImage(const bool blocking = false);
#endif
    ErrorCode computeCoverage();

    ErrorCode initializePipeline(const std::shared_ptr<dai::Device> daiDevice);

    ErrorCode doWork(std::chrono::steady_clock::time_point& previousLoadingAndCalibrationTime);

    ErrorCode evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control);

    void computeMetrics(const CalibrationHandler& handler);
    ConnectedSensor* findConnectedSensor(CameraBoardSocket socket);
    const ConnectedSensor* findConnectedSensor(CameraBoardSocket socket) const;
    /**
     * From dai::CalibrationHandler data convert to DCL dcl::CameraCalibrationHandle, which includes all necesarry data for calibration
     * @return dcl::CameraCalibrationHanlder
     */
    /**
     * Overwrites the internal calibration of DCL with new Calibration data provided by node.
     */
    void setCalibration(CalibrationHandler& handler, bool flash);

    /**
     * DAI held properties
     */
    CalibrationHandler calibrationHandler;
    ImgTransformation imgTransformationA;
    ImgTransformation imgTransformationB;

    std::variant<CameraBoardSocket, HousingCoordinateSystem> daiSocketBase = CameraBoardSocket::CAM_A;
    CameraBoardSocket daiSocketLeft = CameraBoardSocket::CAM_B;
    CameraBoardSocket daiSocketRight = CameraBoardSocket::CAM_C;
    std::pair<int, int> resolutionLeft;
    std::pair<int, int> resolutionRight;
    std::shared_ptr<::spdlog::async_logger> logger;

    // std::chrono::milliseconds sleepingTime = 250ms;
    // static constexpr std::chrono::milliseconds kSleepingTime{250};
    std::chrono::milliseconds sleepingTime{250};
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;
    bool slept = false;
    std::optional<DynamicCalibrationControl::Commands::StartCalibration> startCalibrationCommand;

    /**
     * Calibration state machine, which holds the state of Node and provides a stable environment;
     * - Initialization of pipeline,
     * - Loading images in DCL,
     * - Starting Calibration Check,
     * - Starting of Calibration
     * - Resetting of data
     */
    bool runOnHostVar = true;

    // When old calibration is encountered, issue a warning once
    bool oldCalibrationWarningIssued = false;
};

}  // namespace node
}  // namespace dai
