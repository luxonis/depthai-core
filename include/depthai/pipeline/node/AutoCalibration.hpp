#pragma once

#include <cstdint>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/AutoCalibrationResult.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationControl.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationResults.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/DynamicCalibrationNode.hpp>
#include <depthai/pipeline/node/Gate.hpp>
#include <depthai/properties/AutoCalibrationProperties.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>

#include "depthai/device/Platform.hpp"

namespace spdlog {
class async_logger;
}
namespace dai {
namespace node {

class AutoCalibration : public DeviceNodeCRTP<DeviceNode, AutoCalibration, AutoCalibrationProperties>, public HostRunnable {
   private:
    Subnode<node::DynamicCalibration> dynamicCalibration{*this, "dynamicCalibration"};

    Subnode<node::Sync> sync{*this, "sync"};

    Subnode<node::Gate> gate{*this, "gate"};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    // The firmware instantiates the Sync subnode as a separate pipeline node, so the input map only
    // exists on the host side of the node.
    InputMap& inputs = sync->inputs;
#endif

    std::string leftInputName = "left";
    std::string rightInputName = "right";

   public:
    using DCC = dai::DynamicCalibrationControl;

    std::shared_ptr<AutoCalibration> build(const std::shared_ptr<Camera>& cameraLeft, const std::shared_ptr<Camera>& cameraRight);

    constexpr static const char* NAME = "AutoCalibration";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~AutoCalibration() override;

    std::shared_ptr<AutoCalibrationConfig> initialConfig = std::make_shared<AutoCalibrationConfig>();

    void run() override;

    /**
     * Specify whether to run on host or device.
     * By default, the node runs on the host on RVC2 and on the device on RVC4. The DynamicCalibration subnode follows
     * the placement of the node, the Sync and Gate subnodes always run on the device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Whether the node runs on the host. Without an explicit setRunOnHost() call this is decided by the
     * platform of the assigned device: RVC4 runs on the device, everything else on the host.
     */
    bool runOnHost() const override;

    void buildInternal() override;

    void buildStage1() override;

    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::AutoCalibrationResult, false}}}}};

   protected:
    Properties& getProperties() override;

    /**
     * Calibration access used by the node. The host implementation goes through the dai::Device handle; the
     * RVC4 firmware overrides these to use the device pipeline directly.
     */
    virtual CalibrationHandler readCurrentCalibration();
    virtual Platform getPlatform() const;

   private:
    struct TelemetryAggregateState;

    // report for logging purposes
    struct Report {
        double dataConfidence = 0.;
        double calibrationConfidence = 0.;
        double dataQualityAfterRecalibration = 0.;
        unsigned int numIterationPerRecalibration = 0;
        unsigned int numImagesLoadedForSuccessfulRecalibration = 0;
        bool recalibrating = false;
        bool recalibrationPassed = false;
        bool calibrationUpdated = false;
        double elapsedSeconds = 0.;
        double elapsedRecalibrationSeconds = 0.;
        std::vector<std::pair<double, double>> coveragesAcquired;
        std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::vector<float>> pairwiseRotationDifference;
    };
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /**
     * Input left image
     */
    Input& left = inputs[leftInputName];

    /**
     * Input right image
     */
    Input& right = inputs[rightInputName];

    InputMap& syncedInput = sync->inputs;
#endif

    void postBuildStage() override;

    bool validateIncomingData();

    bool checkCalibration();

    void runContinuousMode();

    void runOnStartMode();

    bool updateCalibrationProcess(std::shared_ptr<dai::CalibrationHandler> calibration);

    void logReport(const Report& report, unsigned int iteration) const;

    void logConfig() const;

    std::shared_ptr<dai::CalibrationHandler> getNewCalibration(unsigned int maxNumIteration, Report& report);

    void loadData(unsigned int numImages);
    /// Creates the logger and telemetry state and resolves config defaults. Runs from buildInternal() on the host and
    /// from run() on the device, where the firmware constructs the node without calling buildInternal().
    void ensureInitialized();
    std::shared_ptr<Device> requireDevice() const;

    std::shared_ptr<dai::CalibrationMetrics> getMetrics(const std::shared_ptr<dai::CalibrationHandler>& calibration);

    static void appendTelemetryAggregateProperties(nlohmann::json& properties, const TelemetryAggregateState& telemetryAggregateState);

    void recordAggregateCycle(double elapsedSeconds);

    void recordAggregateRecalibration(bool passed, double elapsedSeconds);

    void recordAggregateValidationMetrics(double dataConfidence, double calibrationConfidence);

    void recordAggregateRotationDifference(const std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::vector<float>>& pairwiseRotationDifference);

    // internal queues
    Output gateControlQueue{*this, {"gateControlQueue", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, false}}}}};

    Output dynamicCalibrationCommandQueue{*this, {"dynamicCalibrationCommandQueue", DEFAULT_GROUP, {{{DatatypeEnum::DynamicCalibrationControl, false}}}}};

    Input dynamicCalibrationQueue{*this, {"dynamicCalibrationQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::DynamicCalibrationResult, true}}}};

    Input metricsQueue{*this, {"metricsQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::CalibrationMetrics, true}}}};

    Input coverageQueue{*this, {"coverageQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::CoverageData, true}}}};

    Input gateOutput{*this, {"gateOutput", DEFAULT_GROUP, false, 1, {{DatatypeEnum::Buffer, true}}}};

    std::optional<bool> runOnHostVar;
    bool initialized = false;
    std::shared_ptr<TelemetryAggregateState> telemetryAggregateState;
    std::uint64_t telemetryAggregateMetricsHandle = 0;

    // logger
    std::shared_ptr<::spdlog::async_logger> logger;
};

}  // namespace node
}  // namespace dai
