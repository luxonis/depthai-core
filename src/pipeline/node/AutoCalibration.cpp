#include "depthai/pipeline/node/AutoCalibration.hpp"

#include <pipeline/ThreadedNodeImpl.hpp>
#include <pipeline/datatype/MessageGroup.hpp>
#include <stdexcept>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "utility/Telemetry.hpp"

namespace dai {
namespace node {

namespace {

const char* autoCalibModeToString(AutoCalibrationConfig::Mode mode) {
    switch(mode) {
        case AutoCalibrationConfig::Mode::ON_START:
            return "ON_START";
        case AutoCalibrationConfig::Mode::CONTINUOUS:
            return "CONTINUOUS";
    }
    return "UNKNOWN";
}

void emitAutoCalibrationOutputSent(const std::shared_ptr<Device>& device, bool passed, double dataConfidence, double calibrationConfidence) {
    if(!device) return;
    dai::utility::Telemetry::getInstance().event(*device,
                                                 "depthai_auto_calibration_output_sent",
                                                 nlohmann::json{
                                                     {"passed", passed},
                                                     {"dataConfidence", dataConfidence},
                                                     {"calibrationConfidence", calibrationConfidence},
                                                 });
}

}  // namespace

constexpr int MAX_FAILS_PER_RECALIBRATION_DEFAULT = 5;
constexpr int GATE_FPS_DEFAULT = 5;
constexpr int BYTES_PER_SECOND_LIMIT_DEFAULT = GATE_FPS_DEFAULT * 1280 * 800 * 2;
constexpr int PACKET_SIZE_DEFAULT = 100000;

bool areLensesWide(std::shared_ptr<Device> device) {
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
    // Define a lambda or a small helper to route to the correct spdlog method
    auto log = [&](const std::string& fmt, auto&&... args) { logger->info(fmt, args...); };

    log("====== AutoCalibration iteration {} / {} ========", iteration, initialConfig->maxIterations);
    log("Iteration time:         {:.2f}s", report.elapsedSeconds);
    log("dataConfidence          {:.4f}     {:.2f}", report.dataConfidence, initialConfig->dataConfidenceThreshold);
    log("calibrationConfidence   {:.4f}     {:.2f}", report.calibrationConfidence, initialConfig->calibrationConfidenceThreshold);

    if(report.calibrationUpdated) {
        log("Calibration successfully updated");
        log("=================================================");
        return;
    }

    if(report.recalibrating) {
        log("recalibration  {}", report.recalibrationPassed ? "Passed" : "Failed");
        if(report.recalibrationPassed) {
            log("    Recalibration time:   {:.2f}s", report.elapsedRecalibrationSeconds);
            log("    number of iterations  {}", report.numIterationPerRecalibration);
            log("    rotation difference   {:.4f}  {:.4f}  {:.4f}",
                report.rotationDifference.at(0),
                report.rotationDifference.at(1),
                report.rotationDifference.at(2));
            log("    dataQuality           {:.4f}", report.dataQualityAfterRecalibration);
        }
        unsigned i = 0;
        for(const auto& coverageData : report.coveragesAcquired) {
            log("    recalibration iteration:");
            log("        {}    coverageAcquired    {:.1f}    dataAcquired    {:.1f}", i + 1, coverageData.first, coverageData.second);
            i += 1;
        }
    } else {
        log("Recalibration  not triggered");
    }
    log("=================================================");
}

void AutoCalibration::logConfig() const {
    auto log = [&](const std::string& fmt, auto&&... args) { logger->info(fmt, args...); };

    log("====== AutoCalibration Configuration ======");
    log("Mode:                   {}", initialConfig->mode == AutoCalibrationConfig::CONTINUOUS ? "CONTINUOUS" : "ON_START");
    log("Sleeping Time:          {}s", initialConfig->sleepingTime);
    log("Calib. Confidence Thr:  {:.2f}", initialConfig->calibrationConfidenceThreshold);
    log("Data Confidence Thr:    {:.2f}", initialConfig->dataConfidenceThreshold);
    log("Max Iterations:         {}", initialConfig->maxIterations);
    log("Max Images/Recalib:     {}", initialConfig->maxImagesPerRecalibration);
    log("Validation Set Size:    {}", initialConfig->validationSetSize);
    log("Flash Calibration:      {}", initialConfig->flashCalibration ? "Yes" : "No");
    log("===========================================");
}

AutoCalibration::~AutoCalibration() = default;

AutoCalibration::Properties& AutoCalibration::getProperties() {
    return properties;
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

std::shared_ptr<dai::CalibrationMetrics> AutoCalibration::getMetrics(std::shared_ptr<dai::CalibrationHandler> calibration) {
    dynamicCalibrationCommandQueue.send(DCC::computeCalibrationMetrics(*calibration));
    return metricsQueue.get<dai::CalibrationMetrics>();
}

void AutoCalibration::buildInternal() {
    logger = pimpl->logger;
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
                    report.dataQualityAfterRecalibration = dynCalibrationResult->calibrationData.value().dataConfidence;
                    report.recalibrationPassed = true;
                    report.rotationDifference = dynCalibrationResult->calibrationData.value().calibrationDifference.rotationChange;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedRecalibrationSeconds = elapsed.count();
                    report.coveragesAcquired.push_back({coverage->coverageAcquired, coverage->dataAcquired});
                    if(device) {
                        dai::utility::Telemetry::getInstance().event(
                            *device,
                            "depthai_auto_calibration_recalibration_result",
                            nlohmann::json{
                                {"passed", true},
                                {"numIterationPerRecalibration", report.numIterationPerRecalibration},
                                {"elapsedRecalibrationSeconds", report.elapsedRecalibrationSeconds},
                                {"dataQualityAfterRecalibration", report.dataQualityAfterRecalibration},
                                {"rotationDifference",
                                 nlohmann::json::array({report.rotationDifference.at(0), report.rotationDifference.at(1), report.rotationDifference.at(2)})},
                            });
                    }
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
    if(device) {
        dai::utility::Telemetry::getInstance().event(
            *device,
            "depthai_auto_calibration_recalibration_result",
            nlohmann::json{
                {"passed", false},
                {"numIterationPerRecalibration", report.numIterationPerRecalibration},
                {"elapsedRecalibrationSeconds", report.elapsedRecalibrationSeconds},
                {"dataQualityAfterRecalibration", report.dataQualityAfterRecalibration},
            });
    }
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

    auto updateStartTime = std::chrono::steady_clock::now();
    double lastDataConfidence = 0.0;
    double lastCalibrationConfidence = 0.0;
    bool recalibrationTriggered = false;
    bool thresholdsPassed = false;
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
            if(report.recalibrating) recalibrationTriggered = true;
            lastDataConfidence = report.dataQualityAfterRecalibration;
            if(newCalibration) {
                dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*newCalibration, initialConfig->flashCalibration));
                auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., true, *newCalibration);
                emitAutoCalibrationOutputSent(device, true, 0.0, 0.0);
                output.send(resultOutput);
                report.calibrationUpdated = true;
                auto endTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = endTime - startTime;
                report.elapsedSeconds = elapsed.count();  // Set duration
                logReport(report, numIterations + 1);

                if(device) {
                    const auto totalElapsed =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - updateStartTime).count();
                    dai::utility::Telemetry::getInstance().event(*device,
                                                                 "depthai_auto_calibration_calibration_update_result",
                                                                 nlohmann::json{
                                                                     {"updated", true},
                                                                     {"passed", true},
                                                                     {"elapsedSeconds", totalElapsed},
                                                                     {"numIterationsUsed", numIterations + 1},
                                                                     {"dataConfidence", report.dataQualityAfterRecalibration},
                                                                     {"calibrationConfidence", report.calibrationConfidence},
                                                                     {"thresholdsPassed", true},
                                                                     {"recalibration_triggered", recalibrationTriggered},
                                                                     {"flashCalibration", initialConfig->flashCalibration},
                                                                 });
                }
                return true;
            }
        } else {
            logger->info("=== AutoCalibration: loading validation data ({}) ---", initialConfig->validationSetSize);
            loadData(static_cast<unsigned int>(initialConfig->validationSetSize));
            auto metrics = getMetrics(calibration);
            report.dataConfidence = metrics->dataConfidence;
            report.calibrationConfidence = metrics->calibrationConfidence;
            lastDataConfidence = metrics->dataConfidence;
            lastCalibrationConfidence = metrics->calibrationConfidence;
            thresholdsPassed = metrics->dataConfidence > initialConfig->dataConfidenceThreshold
                               && metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold;
            if(metrics->dataConfidence > initialConfig->dataConfidenceThreshold) {
                if(metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold) {
                    dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*calibration, initialConfig->flashCalibration));
                    auto resultOutput = std::make_shared<AutoCalibrationResult>(metrics->dataConfidence, metrics->calibrationConfidence, true, *calibration);
                    emitAutoCalibrationOutputSent(device, true, metrics->dataConfidence, metrics->calibrationConfidence);
                    output.send(resultOutput);
                    report.calibrationUpdated = true;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedSeconds = elapsed.count();  // Set duration
                    logReport(report, numIterations + 1);

                    if(device) {
                        const auto totalElapsed =
                            std::chrono::duration<double>(std::chrono::steady_clock::now() - updateStartTime).count();
                        dai::utility::Telemetry::getInstance().event(*device,
                                                                     "depthai_auto_calibration_calibration_update_result",
                                                                     nlohmann::json{
                                                                         {"updated", true},
                                                                         {"passed", true},
                                                                         {"elapsedSeconds", totalElapsed},
                                                                         {"numIterationsUsed", numIterations + 1},
                                                                         {"dataConfidence", metrics->dataConfidence},
                                                                         {"calibrationConfidence", metrics->calibrationConfidence},
                                                                         {"thresholdsPassed", true},
                                                                         {"recalibration_triggered", recalibrationTriggered},
                                                                         {"flashCalibration", initialConfig->flashCalibration},
                                                                     });
                    }
                    return true;
                }
                auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT, report);
                report.recalibrating = true;
                recalibrationTriggered = true;
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
        emitAutoCalibrationOutputSent(device, false, 0.0, 0.0);
        output.send(resultOutput);
    }

    if(device) {
        const auto totalElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - updateStartTime).count();
        dai::utility::Telemetry::getInstance().event(*device,
                                                     "depthai_auto_calibration_calibration_update_result",
                                                     nlohmann::json{
                                                         {"updated", false},
                                                         {"passed", false},
                                                         {"elapsedSeconds", totalElapsed},
                                                         {"numIterationsUsed", numIterations},
                                                         {"dataConfidence", lastDataConfidence},
                                                         {"calibrationConfidence", lastCalibrationConfidence},
                                                         {"thresholdsPassed", thresholdsPassed},
                                                         {"recalibration_triggered", recalibrationTriggered},
                                                         {"flashCalibration", initialConfig->flashCalibration},
                                                     });
    }
    return false;
}

void AutoCalibration::runContinuousMode() {
    while(mainLoop()) {
        auto startTime = std::chrono::steady_clock::now();
        updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
        auto endTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        logger->info("AutoCalibration update took: {:.2f}s", elapsed.count());
        if(device) {
            dai::utility::Telemetry::getInstance().event(*device,
                                                         "depthai_auto_calibration_cycle_duration",
                                                         nlohmann::json{
                                                             {"elapsedSeconds", elapsed.count()},
                                                             {"mode", autoCalibModeToString(initialConfig->mode)},
                                                         });
        }
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
    updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    logger->info("AutoCalibration update took: {:.2f}s", elapsed.count());
    if(device) {
        dai::utility::Telemetry::getInstance().event(*device,
                                                     "depthai_auto_calibration_cycle_duration",
                                                     nlohmann::json{
                                                         {"elapsedSeconds", elapsed.count()},
                                                         {"mode", autoCalibModeToString(initialConfig->mode)},
                                                     });
    }
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
                logger->warn("AutoCalibration: Not initialized - currently supports only sensors with same resolutions.");
                return false;
            }
            const auto h = leftImgFrame->getHeight();
            const auto w = leftImgFrame->getWidth();

            const bool supportedResolution = (h == 800 && w == 1280) || (h == 400 && w == 640);

            if(!supportedResolution) {
                logger->warn(
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

    if(device) {
        dai::utility::Telemetry::getInstance().event(*device,
                                                     "depthai_auto_calibration_node_started",
                                                     nlohmann::json{
                                                         {"mode", autoCalibModeToString(initialConfig->mode)},
                                                         {"sleepingTime", initialConfig->sleepingTime},
                                                         {"maxIterations", initialConfig->maxIterations},
                                                         {"maxImagesPerRecalibration", initialConfig->maxImagesPerRecalibration},
                                                         {"validationSetSize", initialConfig->validationSetSize},
                                                         {"dataConfidenceThreshold", initialConfig->dataConfidenceThreshold},
                                                         {"calibrationConfidenceThreshold", initialConfig->calibrationConfidenceThreshold},
                                                         {"flashCalibration", initialConfig->flashCalibration},
                                                     });
    }

    if(!validateIncomingData()) {
        return;
    }
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
