#pragma once

#include <chrono>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <deque>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/common/WorldConstraints.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#include "depthai/utility/spimpl.h"

namespace spdlog {
class async_logger;
}
namespace dcl {
class CameraSensorHandle;
}
namespace dai {
namespace node {

/**
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 *
 * The node runs on the host by default. On RVC4 it can run on the device with setRunOnHost(false), so no
 * camera frames cross the link: only control and result messages are exchanged with the host.
 *
 * Calibrated cameras retain their DCL-produced poses relative to housing when available.
 * Other cameras preserve their measured factory transforms from the default stereo-depth reference camera
 * of the first device stereo pair,
 * regardless of calibration chain order. That camera must be a calibration input. Factory
 * calibration is required when any cameras in the handler are not calibration inputs.
 * Housing extrinsics are updated to preserve the calibrated poses. This also applies
 * when the node is used internally by AutoCalibration.
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DynamicCalibration";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~DynamicCalibration() override;

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
    std::string leftInputName = "left";
    std::string rightInputName = "right";
    std::string rgbInputName = "rgb";
    std::vector<std::string> names;
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    // The firmware instantiates the Sync subnode as a separate pipeline node, so the input map only
    // exists on the host side of the node.
    InputMap& inputs = sync->inputs;
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
#endif

    /**
     * Specify whether to run on host or device.
     * By default, the node runs on the host. Running on the device is supported only on RVC4.
     * The Sync subnode follows the placement of the node.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Threads the dynamic calibration library computes on: counts, nice value and CPU affinity. The library's
     * threads are process-wide, so the policy reaches every DynamicCalibration and AutoCalibration node in the
     * same process (the device firmware or the host application). It is applied when the node starts; fields
     * left empty keep the library defaults.
     */
    void setThreadPolicy(const DynamicCalibrationThreadPolicy& policy);

    /**
     * Thread policy set with setThreadPolicy(); all fields are empty unless set.
     */
    DynamicCalibrationThreadPolicy getThreadPolicy() const;

    void postBuildStage() override;

    /**
     * Refine camera orientations using measured marker distances and printed sizes, preserving the resulting
     * camera centers and baseline lengths. Markers are detected in every image set; a set is loaded into the
     * calibration only when every marker the constraints refer to is seen by at least two cameras, otherwise it
     * is skipped and the CoverageData message on coverageOutput says which markers are missing (its `markers`
     * block). The observations of the accepted sets (the latest 50, cleared by ResetData) are handed to the
     * calibration together with the image features. A distance to a socket that is not one of the image
     * inputs uses that socket's current calibration. Set before the pipeline is built.
     */
    void setWorldConstraints(const WorldConstraints& constraints);
    /// Same as setWorldConstraints() with the JSON form described in WorldConstraints.
    void setWorldConstraintsFromJson(const std::string& json);
    /// Same as setWorldConstraints() with the JSON form described in WorldConstraints read from a file.
    void setWorldConstraintsFromFile(const std::filesystem::path& path);
    /// Remove the marker constraints again. Takes effect before the pipeline is built.
    void clearWorldConstraints();
    /// The marker constraints set with setWorldConstraints(), or none.
    std::optional<WorldConstraints> getWorldConstraints() const;

   protected:
    Properties& getProperties() override;

    /**
     * Device access that has no dai::Device equivalent inside the firmware. The host implementation goes
     * through the dai::Device handle; the RVC4 firmware overrides these to use the EEPROM directly.
     */
    virtual CalibrationHandler readFactoryCalibration();
    virtual std::vector<StereoPair> getStereoPairs();
    /// Product and board name as stored in the EEPROM, used to detect modular (FFC) boards.
    virtual std::pair<std::string, std::string> getProductAndBoardName();
    /// Install a calibration at runtime and optionally persist it to the EEPROM user area.
    virtual void applyCalibration(const CalibrationHandler& handler, bool flash);

   private:
    struct ConnectedSensor {
        CameraBoardSocket socket;
        std::vector<std::vector<float>> intrinsics;
        std::vector<float> distortion;
        CameraModel distortionModel = CameraModel::Perspective;
        std::pair<int, int> resolution;
        std::shared_ptr<dcl::CameraSensorHandle> sensorDcl;
    };

    std::vector<ConnectedSensor> connectedSensors;
    std::vector<CameraBoardSocket> socketsInHandler;
    /// Sockets a marker distance refers to that are not image inputs: registered with DCL for their pose only.
    std::vector<ConnectedSensor> auxiliarySensors;
    /// Marker centers of the loaded image sets, newest last.
    std::deque<MarkerObservationSet> markerObservations;
    /// Marker status of the last image set that was looked at, accepted or not; attached to the CoverageData messages.
    std::optional<CoverageData::MarkerCoverage> lastMarkerCoverage;
    /// The last CoverageData computed by DCL; its numbers are repeated when an image set is rejected for missing markers.
    std::shared_ptr<CoverageData> lastCoverageMessage;
    std::optional<CameraBoardSocket> leftQueueSocket;
    std::optional<CameraBoardSocket> rightQueueSocket;

    class Impl;
    spimpl::impl_ptr<Impl> pimplDCL;
    /// Hands properties.threadPolicy to the library; logs and keeps going when the library rejects it.
    void applyThreadPolicy();
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
        /// World constraints are configured and the image set did not show every required marker to two cameras.
        MARKERS_NOT_VISIBLE = 10,
    };

    /**
     * Check if the node is set to run on host
     */

    void run() override;

    ErrorCode runCalibration(const dai::CalibrationHandler& calibHandler, const bool force = false, const bool keepCameraCenters = true);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    ErrorCode runLoadImage(const bool blocking = false);
#endif
    ErrorCode computeCoverage();
    /// Sends the CoverageData (last DCL numbers, current marker status) and a DynamicCalibrationResult for an
    /// image set rejected because of missing markers, so every loaded or skipped set produces both messages.
    void reportMarkersNotVisible();

    ErrorCode initializePipeline();

    /// Creates the DCL state and logger. Runs from buildInternal() on the host and from run() on the device,
    /// where the firmware constructs the node without calling buildInternal().
    void ensureInitialized();

    ErrorCode doWork(std::chrono::steady_clock::time_point& previousLoadingAndCalibrationTime);

    ErrorCode evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control);

    void computeMetrics(const CalibrationHandler& handler);
    const ConnectedSensor* findConnectedSensor(CameraBoardSocket socket) const;
    /// Registers the sockets that marker distances refer to but that deliver no images.
    ErrorCode initializeAuxiliarySensors(const EepromData& eepromData);
    /**
     * From dai::CalibrationHandler data convert to DCL dcl::CameraCalibrationHandle, which includes all necesarry data for calibration
     * @return dcl::CameraCalibrationHanlder
     */
    /**
     * Overwrites the internal calibration of DCL with new Calibration data provided by node.
     */
    void setCalibration(CalibrationHandler& handler, bool flash);

    /// Platform the node runs for: the device's on the host, RVC4 inside the RVC4 firmware (no device handle there).
    Platform getPlatform() const;

    /**
     * DAI held properties
     */
    CalibrationHandler calibrationHandler;
    ImgTransformation imgTransformationA;
    ImgTransformation imgTransformationB;

    std::variant<CameraBoardSocket, HousingCoordinateSystem> daiSocketBase = CameraBoardSocket::CAM_A;
    std::shared_ptr<::spdlog::async_logger> logger;

    // std::chrono::milliseconds sleepingTime = 250ms;
    // static constexpr std::chrono::milliseconds kSleepingTime{250};
    std::chrono::milliseconds sleepingTime{250};
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;
    bool slept = false;
    std::optional<DynamicCalibrationControl::Commands::StartCalibration> startCalibrationCommand;

    /// Placement set with setRunOnHost(): host by default, the device only on RVC4.
    bool runOnHostVar = true;

    // When old calibration is encountered, issue a warning once
    bool oldCalibrationWarningIssued = false;

    // When the marker detector rejects an image, issue a warning once
    bool markerDetectionWarningIssued = false;
    // Configured markers already reported as too small to use
    std::set<int32_t> markerSizeWarningIssued;
    // A calibration without any applicable marker constraint has been reported
    bool metricReferenceWarningIssued = false;
    /// Reports a calibration that runs without its configured metric reference: once as a warning, then at debug level.
    void warnMetricReferenceMissing(const std::string& reason);
    // A calibration rejected because the marker constraints do not fit the scene has been reported
    bool constraintMismatchWarningIssued = false;
    /// Reports a calibration rejected for its marker constraints: once as a warning, then at debug level.
    void warnConstraintMismatch(const std::string& reason);
    /// Start of the current run of image sets rejected for missing markers, and when it was last reported.
    std::optional<std::chrono::steady_clock::time_point> markersMissingSince;
    std::optional<std::chrono::steady_clock::time_point> markersMissingReportedAt;
    /// Keeps a run of image sets rejected for missing markers from going unnoticed: warns once the run lasts
    /// kMarkersMissingWarnAfter and then every kMarkersMissingWarnPeriod; accepted tells that the run ended.
    void trackMissingMarkers(bool accepted);
};

}  // namespace node
}  // namespace dai
