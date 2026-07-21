#include "depthai/pipeline/node/AutoCalibration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <pipeline/ThreadedNodeImpl.hpp>
#include <pipeline/datatype/MessageGroup.hpp>
#include <stdexcept>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "utility/Telemetry.hpp"

namespace dai {
namespace node {

namespace {

void addAutoCalibrationAggregateRotationDifference(
    nlohmann::json& properties, const std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::array<double, 3>>& pairwiseRotationDifferenceSums) {
    auto pairwiseRotationDifferenceJson = nlohmann::json::array();
    for(const auto& [socketPair, rotationSum] : pairwiseRotationDifferenceSums) {
        pairwiseRotationDifferenceJson.push_back({{"socket_a", dai::toString(socketPair.first)},
                                                  {"socket_b", dai::toString(socketPair.second)},
                                                  {"rotation_difference_sum", {rotationSum[0], rotationSum[1], rotationSum[2]}}});
    }
    properties["auto_calibration_pairwise_rotation_difference_sums"] = std::move(pairwiseRotationDifferenceJson);
}

}  // namespace

struct AutoCalibration::TelemetryAggregateState {
    std::mutex mutex;
    std::uint64_t cyclesCompleted = 0;
    std::uint64_t recalibrationTriggered = 0;
    std::uint64_t recalibrationSucceeded = 0;
    std::uint64_t recalibrationFailed = 0;
    std::uint64_t calibrationUpdated = 0;
    std::uint64_t outputSent = 0;
    double cycleDurationSum = 0.0;
    double recalibrationDurationSum = 0.0;
    double latestCycleDuration = 0.0;
    double dataConfidenceSum = 0.0;
    std::uint64_t dataConfidenceSamples = 0;
    double calibrationConfidenceSum = 0.0;
    std::uint64_t calibrationConfidenceSamples = 0;
    std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::array<double, 3>> pairwiseRotationDifferenceSums;
};

const char* telemetryAutoCalibrationModeName(const AutoCalibrationConfig::Mode mode) {
    switch(mode) {
        case AutoCalibrationConfig::Mode::ON_START:
            return "ON_START";
        case AutoCalibrationConfig::Mode::CONTINUOUS:
            return "CONTINUOUS";
    }
    return "UNKNOWN";
}

constexpr int MAX_FAILS_PER_RECALIBRATION_DEFAULT = 5;
constexpr int GATE_FPS_DEFAULT = 5;
constexpr int BYTES_PER_SECOND_LIMIT_DEFAULT = GATE_FPS_DEFAULT * 1280 * 800 * 2;
constexpr int PACKET_SIZE_DEFAULT = 100000;

bool areLensesWide(const std::shared_ptr<Device>& device) {
    auto handler = device->getCalibration();
    auto eepromData = handler.getEepromData();
    const auto& hardwareConf = eepromData.hardwareConf;
    size_t start = 0;

    while(start < hardwareConf.size()) {
        const size_t end = hardwareConf.find('-', start);
        const std::string token = hardwareConf.substr(start, end == std::string::npos ? std::string::npos : end - start);

        if(token == "FV01" || token == "FV05") {
            return true;
        }
        if(token.rfind("FV", 0) == 0) {
            return false;
        }

        if(end == std::string::npos) break;
        start = end + 1;
    }
    return false;
}

void AutoCalibration::logReport(const Report& report, unsigned int iteration) const {
    logger->info("====== AutoCalibration iteration {} / {} ========", iteration, initialConfig->maxIterations);
    logger->info("Iteration time:         {:.2f}s", report.elapsedSeconds);
    logger->info("dataConfidence          {:.4f}     {:.2f}", report.dataConfidence, initialConfig->dataConfidenceThreshold);
    logger->info("calibrationConfidence   {:.4f}     {:.2f}", report.calibrationConfidence, initialConfig->calibrationConfidenceThreshold);

    if(report.calibrationUpdated) {
        logger->info("Calibration successfully updated");
        logger->info("=================================================");
        return;
    }

    if(report.recalibrating) {
        logger->info("recalibration  {}", report.recalibrationPassed ? "Passed" : "Failed");
        if(report.recalibrationPassed) {
            logger->info("    Recalibration time:   {:.2f}s", report.elapsedRecalibrationSeconds);
            logger->info("    number of iterations  {}", report.numIterationPerRecalibration);
            for(const auto& [socketPair, rotationDifference] : report.pairwiseRotationDifference) {
                if(rotationDifference.size() < 3) continue;
                logger->info("    pairwise rotation difference {} -> {}   {:.4f}  {:.4f}  {:.4f}",
                             toString(socketPair.first),
                             toString(socketPair.second),
                             rotationDifference[0],
                             rotationDifference[1],
                             rotationDifference[2]);
            }
            logger->info("    dataQuality           {:.4f}", report.dataQualityAfterRecalibration);
        }
        unsigned i = 0;
        for(const auto& coverageData : report.coveragesAcquired) {
            logger->info("    recalibration iteration:");
            logger->info("        {}    coverageAcquired    {:.1f}    dataAcquired    {:.1f}", i + 1, coverageData.first, coverageData.second);
            i += 1;
        }
    } else {
        logger->info("Recalibration  not triggered");
    }
    logger->info("=================================================");
}

void AutoCalibration::logConfig() const {
    logger->info("====== AutoCalibration Configuration ======");
    logger->info("Mode:                   {}", initialConfig->mode == AutoCalibrationConfig::CONTINUOUS ? "CONTINUOUS" : "ON_START");
    logger->info("Sleeping Time:          {}s", initialConfig->sleepingTime);
    logger->info("Calib. Confidence Thr:  {:.2f}", initialConfig->calibrationConfidenceThreshold);
    logger->info("Data Confidence Thr:    {:.2f}", initialConfig->dataConfidenceThreshold);
    logger->info("Max Iterations:         {}", initialConfig->maxIterations);
    logger->info("Max Images/Recalib:     {}", initialConfig->maxImagesPerRecalibration);
    logger->info("Validation Set Size:    {}", initialConfig->validationSetSize);
    logger->info("Flash Calibration:      {}", initialConfig->flashCalibration ? "Yes" : "No");
    logger->info("===========================================");
}

AutoCalibration::~AutoCalibration() {
    if(telemetryAggregateMetricsHandle != 0) {
        utility::Telemetry::getInstance().removeAggregateMetrics(telemetryAggregateMetricsHandle);
        telemetryAggregateMetricsHandle = 0;
    }
}

AutoCalibration::Properties& AutoCalibration::getProperties() {
    return properties;
}

void AutoCalibration::appendTelemetryAggregateProperties(nlohmann::json& properties, const TelemetryAggregateState& telemetryAggregateState) {
    properties["auto_calibration_cycles_completed"] = telemetryAggregateState.cyclesCompleted;
    properties["auto_calibration_recalibration_triggered"] = telemetryAggregateState.recalibrationTriggered;
    properties["auto_calibration_recalibration_succeeded"] = telemetryAggregateState.recalibrationSucceeded;
    properties["auto_calibration_recalibration_failed"] = telemetryAggregateState.recalibrationFailed;
    properties["auto_calibration_calibration_updated"] = telemetryAggregateState.calibrationUpdated;
    properties["auto_calibration_output_sent"] = telemetryAggregateState.outputSent;
    properties["auto_calibration_cycle_duration_latest_s"] = telemetryAggregateState.latestCycleDuration;
    properties["auto_calibration_cycle_duration_avg_s"] =
        telemetryAggregateState.cyclesCompleted == 0 ? 0.0 : telemetryAggregateState.cycleDurationSum / telemetryAggregateState.cyclesCompleted;
    const auto recalibrationCount = telemetryAggregateState.recalibrationSucceeded + telemetryAggregateState.recalibrationFailed;
    properties["auto_calibration_recalibration_duration_avg_s"] =
        recalibrationCount == 0 ? 0.0 : telemetryAggregateState.recalibrationDurationSum / static_cast<double>(recalibrationCount);
    properties["auto_calibration_avg_data_confidence"] =
        telemetryAggregateState.dataConfidenceSamples == 0
            ? 0.0
            : telemetryAggregateState.dataConfidenceSum / static_cast<double>(telemetryAggregateState.dataConfidenceSamples);
    properties["auto_calibration_avg_calibration_confidence"] =
        telemetryAggregateState.calibrationConfidenceSamples == 0
            ? 0.0
            : telemetryAggregateState.calibrationConfidenceSum / static_cast<double>(telemetryAggregateState.calibrationConfidenceSamples);
    addAutoCalibrationAggregateRotationDifference(properties, telemetryAggregateState.pairwiseRotationDifferenceSums);
}

void AutoCalibration::recordAggregateCycle(double elapsedSeconds) {
    if(!telemetryAggregateState) {
        return;
    }

    std::lock_guard<std::mutex> lock(telemetryAggregateState->mutex);
    telemetryAggregateState->cyclesCompleted += 1;
    telemetryAggregateState->cycleDurationSum += elapsedSeconds;
    telemetryAggregateState->latestCycleDuration = elapsedSeconds;
}

void AutoCalibration::recordAggregateRecalibration(bool passed, double elapsedSeconds) {
    if(!telemetryAggregateState) {
        return;
    }

    std::lock_guard<std::mutex> lock(telemetryAggregateState->mutex);
    if(passed) {
        telemetryAggregateState->recalibrationSucceeded += 1;
    } else {
        telemetryAggregateState->recalibrationFailed += 1;
    }
    telemetryAggregateState->recalibrationDurationSum += elapsedSeconds;
}

void AutoCalibration::recordAggregateValidationMetrics(double dataConfidence, double calibrationConfidence) {
    if(!telemetryAggregateState) {
        return;
    }

    std::lock_guard<std::mutex> lock(telemetryAggregateState->mutex);
    telemetryAggregateState->dataConfidenceSum += dataConfidence;
    telemetryAggregateState->dataConfidenceSamples += 1;
    telemetryAggregateState->calibrationConfidenceSum += calibrationConfidence;
    telemetryAggregateState->calibrationConfidenceSamples += 1;
}

void AutoCalibration::recordAggregateRotationDifference(
    const std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::vector<float>>& pairwiseRotationDifference) {
    if(!telemetryAggregateState) {
        return;
    }

    std::lock_guard<std::mutex> lock(telemetryAggregateState->mutex);
    for(const auto& [socketPair, rotationDifference] : pairwiseRotationDifference) {
        auto& rotationSum = telemetryAggregateState->pairwiseRotationDifferenceSums[socketPair];
        const auto copyCount = std::min<std::size_t>(3, rotationDifference.size());
        for(std::size_t axis = 0; axis < copyCount; ++axis) {
            rotationSum[axis] += static_cast<double>(rotationDifference[axis]);
        }
    }
}

void AutoCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

void addPoolsForAutoCalibration(const std::shared_ptr<Camera>& camera, int additionalPools) {
    auto numIspPool = camera->getIspNumFramesPool();
    if(numIspPool > 0) {
        camera->setIspNumFramesPool(numIspPool + additionalPools);
    } else {
        // If getIspNumFramesPool() is unavailable/non-positive, use a known-safe baseline via setIspNumFramesPool().
        camera->setIspNumFramesPool(6);
    }
}

std::shared_ptr<AutoCalibration> AutoCalibration::build(const std::shared_ptr<Camera>& cameraLeft, const std::shared_ptr<Camera>& cameraRight) {
    if(!cameraLeft || !cameraRight) {
        throw std::invalid_argument("AutoCalibration::build requires non-null camera pointers");
    }

    sync->setRunOnHost(false);
    gate->setRunOnHost(false);
    auto outputCameraLeft = cameraLeft->requestIspOutput();
    auto outputCameraRight = cameraRight->requestIspOutput();
    outputCameraLeft->link(left);
    outputCameraRight->link(right);
    sync->out.link(gate->input);
    gate->output.link(dynamicCalibration->syncInput);
    if(device->getPlatform() == Platform::RVC2) {
        addPoolsForAutoCalibration(cameraLeft, 3);
        addPoolsForAutoCalibration(cameraRight, 3);
    }

    dynamicCalibrationCommandQueue.link(dynamicCalibration->inputControl);
    gateControlQueue.link(gate->inputControl);

    dynamicCalibration->calibrationOutput.link(dynamicCalibrationQueue);
    dynamicCalibration->coverageOutput.link(coverageQueue);
    dynamicCalibration->metricsOutput.link(metricsQueue);
    gate->output.link(gateOutput);

    gate->initialConfig->open = false;
    gate->initialConfig->fps = GATE_FPS_DEFAULT;
    dynamicCalibration->syncInput.setMaxSize(1);
    return std::static_pointer_cast<AutoCalibration>(shared_from_this());
}

/**
 * Check if the node is set to run on host
 */
bool AutoCalibration::runOnHost() const {
    return runOnHostVar;
}

void AutoCalibration::postBuildStage() {
    auto xlinkBridge = gate->output.getXLinkBridge();
    if(!xlinkBridge || !xlinkBridge->xLinkOut) {
        logger->warn("AutoCalibration: missing XLink bridge; skipping output throttling setup.");
        return;
    }
    xlinkBridge->xLinkOut->setBytesPerSecondLimit(BYTES_PER_SECOND_LIMIT_DEFAULT);
    xlinkBridge->xLinkOut->setPacketSize(PACKET_SIZE_DEFAULT);
    xlinkBridge->xLinkOut->input.setMaxSize(1);
    xlinkBridge->xLinkOut->input.setBlocking(false);
}

void AutoCalibration::loadData(unsigned int numImages) {
    gateControlQueue.send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    coverageQueue.tryGetAll<dai::CoverageData>();
    for(unsigned int i = 0; i < numImages; i++) {
        dynamicCalibrationCommandQueue.send(DCC::loadImage());
        coverageQueue.get<dai::CoverageData>();  // wait until the data are loaded
    }
    gateControlQueue.send(dai::GateControl::closeGate());
    dynamicCalibration->syncInput.tryGetAll<dai::MessageGroup>();
}

std::shared_ptr<dai::CalibrationMetrics> AutoCalibration::getMetrics(const std::shared_ptr<dai::CalibrationHandler>& calibration) {
    dynamicCalibrationCommandQueue.send(DCC::computeCalibrationMetrics(*calibration));
    return metricsQueue.get<dai::CalibrationMetrics>();
}

void AutoCalibration::buildInternal() {
    logger = pimpl->logger;
    telemetryAggregateState = std::make_shared<TelemetryAggregateState>();
    auto telemetryState = telemetryAggregateState;
    telemetryAggregateMetricsHandle = utility::Telemetry::getInstance().addAggregateMetrics([telemetryState](nlohmann::json& properties) {
        std::lock_guard<std::mutex> lock(telemetryState->mutex);
        appendTelemetryAggregateProperties(properties, *telemetryState);
    });
    if(initialConfig->dataConfidenceThreshold < 0.0) {
        initialConfig->dataConfidenceThreshold = areLensesWide(device) ? 0.65 : 0.85;
    }
}

/**
return calibration from DynamicCalibration node
**/
std::shared_ptr<dai::CalibrationHandler> AutoCalibration::getNewCalibration(unsigned int maxNumIteration, Report& report) {
    auto startTime = std::chrono::steady_clock::now();  // Start timer
    gateControlQueue.send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    {
        auto& state = *telemetryAggregateState;
        std::lock_guard<std::mutex> lock(state.mutex);
        state.recalibrationTriggered += 1;
    }
    std::shared_ptr<CoverageData> coverage;
    for(unsigned int i = 0; i < maxNumIteration; i++) {
        logger->info("=== AutoCalibration: START recalib {}/{}", i + 1, maxNumIteration);
        dynamicCalibrationCommandQueue.send(DCC::startCalibration());
        for(unsigned int numLoadedImages = 0; numLoadedImages < initialConfig->maxImagesPerRecalibration; numLoadedImages++) {
            if(!mainLoop()) return nullptr;
            auto dynCalibrationResult = dynamicCalibrationQueue.get<dai::DynamicCalibrationResult>();
            coverage = coverageQueue.get<dai::CoverageData>();
            logger->debug("=== AutoCalibration: MID recalib {}/{} img {}/{} noData info=\"{}\" cov={:.2f} data={:.2f} ===",
                          i + 1,
                          maxNumIteration,
                          numLoadedImages + 1,
                          initialConfig->maxImagesPerRecalibration,
                          dynCalibrationResult->info,
                          coverage ? coverage->coverageAcquired : 0.0f,
                          coverage ? coverage->dataAcquired : 0.0f);
            if(dynCalibrationResult->calibrationData) {
                if(dynCalibrationResult->calibrationData.value().dataConfidence > initialConfig->dataConfidenceThreshold) {
                    gateControlQueue.send(dai::GateControl::closeGate());
                    dynamicCalibration->syncInput.tryGetAll<dai::MessageGroup>();
                    dynamicCalibrationCommandQueue.send(DCC::resetData());
                    report.numIterationPerRecalibration = i + 1;
                    report.numImagesLoadedForSuccessfulRecalibration = numLoadedImages + 1;
                    report.dataQualityAfterRecalibration = dynCalibrationResult->calibrationData.value().dataConfidence;
                    report.recalibrationPassed = true;
                    report.pairwiseRotationDifference = dynCalibrationResult->calibrationData.value().calibrationDifference.pairwiseRotationDifference;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedRecalibrationSeconds = elapsed.count();
                    report.coveragesAcquired.push_back({coverage->coverageAcquired, coverage->dataAcquired});
                    recordAggregateRecalibration(true, report.elapsedRecalibrationSeconds);
                    recordAggregateRotationDifference(report.pairwiseRotationDifference);
                    return std::make_shared<dai::CalibrationHandler>(dynCalibrationResult->calibrationData.value().newCalibration);
                }
                dynamicCalibrationCommandQueue.send(DCC::resetData());
                break;
            }
        }

        // One-shot command semantics: close and reset every attempt.

        if(coverage) {
            report.coveragesAcquired.push_back({coverage->coverageAcquired, coverage->dataAcquired});
        } else {
            report.coveragesAcquired.push_back({0., 0.});
        }
    }
    dynamicCalibrationCommandQueue.send(DCC::stopCalibration());
    report.recalibrationPassed = false;
    report.numIterationPerRecalibration = maxNumIteration;
    gateControlQueue.send(dai::GateControl::closeGate());
    dynamicCalibration->syncInput.tryGetAll<dai::MessageGroup>();
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    report.elapsedRecalibrationSeconds = elapsed.count();
    recordAggregateRecalibration(false, report.elapsedRecalibrationSeconds);
    return nullptr;
}

/**
1. load data
2. if data OK -> check calibration
   if data not OK -> go to 1.
3. if calibration not OK -> recalibrate; go to 1.
   if calibration OK -> set calibration
**/
bool AutoCalibration::updateCalibrationProcess(std::shared_ptr<dai::CalibrationHandler> calibration) {
    if(initialConfig->validationSetSize < 0) {
        throw std::invalid_argument("AutoCalibration: validationSetSize must be non-negative");
    }

    unsigned int numIterations = 0;
    while(numIterations < initialConfig->maxIterations && mainLoop()) {
        auto startTime = std::chrono::steady_clock::now();  // Start timer
        logger->info("=== AutoCalibration: update iteration {}/{} (validationSetSize={}) ---",
                     numIterations + 1,
                     initialConfig->maxIterations,
                     initialConfig->validationSetSize);
        Report report;
        dynamicCalibrationCommandQueue.send(DCC::resetData());
        if(initialConfig->validationSetSize == 0) {
            auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT, report);
            if(newCalibration) {
                dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*newCalibration, initialConfig->flashCalibration));
                auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., true, *newCalibration);
                output.send(resultOutput);
                {
                    auto& state = *telemetryAggregateState;
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.calibrationUpdated += 1;
                    state.outputSent += 1;
                }
                report.calibrationUpdated = true;
                auto endTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = endTime - startTime;
                report.elapsedSeconds = elapsed.count();  // Set duration
                logReport(report, numIterations + 1);
                return true;
            }
        } else {
            logger->info("=== AutoCalibration: loading validation data ({}) ---", initialConfig->validationSetSize);
            loadData(static_cast<unsigned int>(initialConfig->validationSetSize));
            auto metrics = getMetrics(calibration);
            report.dataConfidence = metrics->dataConfidence;
            report.calibrationConfidence = metrics->calibrationConfidence;
            recordAggregateValidationMetrics(report.dataConfidence, report.calibrationConfidence);
            if(metrics->dataConfidence > initialConfig->dataConfidenceThreshold) {
                if(metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold) {
                    dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*calibration, initialConfig->flashCalibration));
                    auto resultOutput = std::make_shared<AutoCalibrationResult>(metrics->dataConfidence, metrics->calibrationConfidence, true, *calibration);
                    output.send(resultOutput);
                    {
                        auto& state = *telemetryAggregateState;
                        std::lock_guard<std::mutex> lock(state.mutex);
                        state.calibrationUpdated += 1;
                        state.outputSent += 1;
                    }
                    report.calibrationUpdated = true;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedSeconds = elapsed.count();  // Set duration
                    logReport(report, numIterations + 1);
                    return true;
                }
                auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT, report);
                report.recalibrating = true;
                if(newCalibration) {
                    calibration = std::move(newCalibration);
                }
            }
        }
        auto endTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        report.elapsedSeconds = elapsed.count();  // Set duration
        logReport(report, numIterations + 1);
        ++numIterations;
    }

    if(mainLoop() && calibration) {
        auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., false, *calibration);
        output.send(resultOutput);
        {
            auto& state = *telemetryAggregateState;
            std::lock_guard<std::mutex> lock(state.mutex);
            state.outputSent += 1;
        }
    }
    return false;
}

void AutoCalibration::runContinuousMode() {
    while(mainLoop()) {
        auto startTime = std::chrono::steady_clock::now();
        const auto passed = updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
        auto endTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        recordAggregateCycle(elapsed.count());
        utility::Telemetry::getInstance().event(
            "depthai_auto_calibration_run_finished",
            nlohmann::json{{"mode", telemetryAutoCalibrationModeName(initialConfig->mode)}, {"passed", passed}, {"elapsed_seconds", elapsed.count()}});
        logger->info("AutoCalibration update took: {:.2f}s", elapsed.count());
        int elapsedSleeping = 0;
        // Continue sleeping only if total time isn't met AND mainLoop is still true
        while(elapsedSleeping < initialConfig->sleepingTime && mainLoop()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            elapsedSleeping += 1;
        }
    }
}

void AutoCalibration::runOnStartMode() {
    auto startTime = std::chrono::steady_clock::now();
    const auto passed = updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    recordAggregateCycle(elapsed.count());
    utility::Telemetry::getInstance().event(
        "depthai_auto_calibration_run_finished",
        nlohmann::json{{"mode", telemetryAutoCalibrationModeName(initialConfig->mode)}, {"passed", passed}, {"elapsed_seconds", elapsed.count()}});
    logger->info("AutoCalibration update took: {:.2f}s", elapsed.count());
}

bool AutoCalibration::validateIncomingData() {
    std::chrono::seconds waitingTime(1);
    bool timedout = true;

    while(mainLoop()) {
        gateControlQueue.send(dai::GateControl::openGate(1, gate->initialConfig->fps));
        auto messageGroup = gateOutput.get<MessageGroup>(waitingTime, timedout);

        if(!timedout && messageGroup) {
            auto leftImgFrame = messageGroup->get<ImgFrame>(leftInputName);
            auto rightImgFrame = messageGroup->get<ImgFrame>(rightInputName);

            if(!leftImgFrame || !rightImgFrame) {
                logger->warn("AutoCalibration: Not initialized - Empty message groups.");
                return false;
            }

            if(leftImgFrame->getWidth() != rightImgFrame->getWidth() || leftImgFrame->getHeight() != rightImgFrame->getHeight()) {
                logger->info("AutoCalibration: Not initialized - currently supports only sensors with same resolutions.");
                return false;
            }
            const auto h = leftImgFrame->getHeight();
            const auto w = leftImgFrame->getWidth();

            const bool supportedResolution = (h == 800 && w == 1280) || (h == 400 && w == 640);

            if(!supportedResolution) {
                logger->info(
                    "AutoCalibration: Not initialized - currently supports only "
                    "1280x800 or 640x400 resolution not {}x{}.",
                    w,
                    h);
                return false;
            }
            return true;
        }
    }
    return false;
}

void AutoCalibration::run() {
    logger->info("AutoCalibration started to work!");
    if(!validateIncomingData()) {
        return;
    }
    utility::Telemetry::getInstance().event("depthai_auto_calibration_node_started",
                                            nlohmann::json{{"mode", telemetryAutoCalibrationModeName(initialConfig->mode)},
                                                           {"sleeping_time_s", initialConfig->sleepingTime},
                                                           {"max_iterations", initialConfig->maxIterations},
                                                           {"max_images_per_recalibration", initialConfig->maxImagesPerRecalibration},
                                                           {"validation_set_size", initialConfig->validationSetSize},
                                                           {"data_confidence_threshold", initialConfig->dataConfidenceThreshold},
                                                           {"calibration_confidence_threshold", initialConfig->calibrationConfidenceThreshold},
                                                           {"flash_calibration", initialConfig->flashCalibration}});
    logConfig();
    switch(initialConfig->mode) {
        case AutoCalibrationConfig::Mode::CONTINUOUS:
            logger->info("Running continuous mode.");
            runContinuousMode();
            break;
        case AutoCalibrationConfig::Mode::ON_START:
            logger->info("Running on-start mode.");
            runOnStartMode();
            logger->info("On-start mode finished.");
            break;
        default:
            logger->info("Incorrect AutoCalibration mode.");
            break;
    }
}

}  // namespace node
}  // namespace dai
