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
    InputMap& inputs = sync->inputs;  // Firmware runs Sync as a separate node: this input map exists only on host
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
    std::vector<ConnectedSensor> auxiliarySensors;        // Marker-distance sockets that are not image inputs; DCL uses only their pose
    std::deque<MarkerObservationSet> markerObservations;  // Newest last
    std::optional<CoverageData::MarkerCoverage> lastMarkerCoverage;
    std::shared_ptr<CoverageData> lastCoverageMessage;
    std::optional<CameraBoardSocket> leftQueueSocket;
    std::optional<CameraBoardSocket> rightQueueSocket;

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
        MARKERS_NOT_VISIBLE = 10,  // World constraints set and a required marker was not seen by two cameras
    };

    void run() override;

    ErrorCode runCalibration(const dai::CalibrationHandler& calibHandler, const bool force = false, const bool keepCameraCenters = true);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    ErrorCode runLoadImage(const bool blocking = false);
#endif
    ErrorCode computeCoverage();
    void reportMarkersNotVisible();  // Every loaded or skipped image set must produce both messages

    ErrorCode initializePipeline();

    void ensureInitialized();  // Firmware constructs the node without buildInternal(), so run() calls it too

    ErrorCode doWork(std::chrono::steady_clock::time_point& previousLoadingAndCalibrationTime);

    ErrorCode evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control);

    void computeMetrics(const CalibrationHandler& handler);
    const ConnectedSensor* findConnectedSensor(CameraBoardSocket socket) const;
    ErrorCode initializeAuxiliarySensors(const EepromData& eepromData);
    void setCalibration(CalibrationHandler& handler, bool flash);

    Platform getPlatform() const;  // RVC4 inside the RVC4 firmware, which has no device handle

    CalibrationHandler calibrationHandler;
    ImgTransformation imgTransformationA;
    ImgTransformation imgTransformationB;

    std::variant<CameraBoardSocket, HousingCoordinateSystem> daiSocketBase = CameraBoardSocket::CAM_A;
    std::shared_ptr<::spdlog::async_logger> logger;

    std::chrono::milliseconds sleepingTime{250};
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;
    bool slept = false;
    std::optional<DynamicCalibrationControl::Commands::StartCalibration> startCalibrationCommand;

    bool runOnHostVar = true;  // Device placement only on RVC4

    bool oldCalibrationWarningIssued = false;

    bool markerDetectionWarningIssued = false;
    std::set<int32_t> markerSizeWarningIssued;
    bool metricReferenceWarningIssued = false;
    void warnMetricReferenceMissing(const std::string& reason);
    bool constraintMismatchWarningIssued = false;
    void warnConstraintMismatch(const std::string& reason);
    std::optional<std::chrono::steady_clock::time_point> markersMissingSince;
    std::optional<std::chrono::steady_clock::time_point> markersMissingReportedAt;
    void trackMissingMarkers(bool accepted);
};

}  // namespace node
}  // namespace dai
