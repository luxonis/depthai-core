#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/AutoCalibrationResult.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationControl.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationResults.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/DynamicCalibrationNode.hpp>
#include <depthai/pipeline/node/Gate.hpp>
#include <depthai/properties/AutoCalibrationProperties.hpp>

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

    InputMap& inputs = sync->inputs;

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

    void setRunOnHost(bool runOnHost);

    bool runOnHost() const override;

    void buildInternal() override;

    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::AutoCalibrationResult, false}}}}};

   protected:
    Properties& getProperties() override;

   private:
    // report for logging purposes
    struct Report {
        double dataConfidence = 0.;
        double calibrationConfidence = 0.;
        double dataQualityAfterRecalibration = 0.;
        unsigned int numIterationPerRecalibration = 0;
        bool recalibrating = false;
        bool recalibrationPassed = false;
        bool calibrationUpdated = false;
        double elapsedSeconds = 0.;
        double elapsedRecalibrationSeconds = 0.;
        std::vector<std::pair<double, double>> coveragesAcquired;
        std::array<float, 3> rotationDifference;
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

    std::shared_ptr<dai::CalibrationMetrics> getMetrics(std::shared_ptr<dai::CalibrationHandler> calibration);

    AutoCalibrationProperties properties;

    // internal queues
    Output gateControlQueue{*this, {"gateControlQueue", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, false}}}}};

    Output dynamicCalibrationCommandQueue{*this, {"dynamicCalibrationCommandQueue", DEFAULT_GROUP, {{{DatatypeEnum::DynamicCalibrationControl, false}}}}};

    Input dynamicCalibrationQueue{*this, {"dynamicCalibrationQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::DynamicCalibrationResult, true}}}};

    Input metricsQueue{*this, {"metricsQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::CalibrationMetrics, true}}}};

    Input coverageQueue{*this, {"coverageQueue", DEFAULT_GROUP, false, 1, {{DatatypeEnum::CoverageData, true}}}};

    Input gateOutput{*this, {"gateOutput", DEFAULT_GROUP, false, 1, {{DatatypeEnum::Buffer, true}}}};

    bool runOnHostVar = true;

    // logger
    std::shared_ptr<::spdlog::async_logger> logger;
};

}  // namespace node
}  // namespace dai
