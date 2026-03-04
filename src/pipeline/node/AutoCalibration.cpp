#include "depthai/pipeline/node/AutoCalibration.hpp"

#include <pipeline/ThreadedNodeImpl.hpp>
#include <pipeline/datatype/MessageGroup.hpp>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"

namespace dai {
namespace node {

#define MAX_FAILS_PER_RECALIBRATION_DEFAULT 5
#define GATE_FPS_DEFAULT 5
#define BYTES_PER_SECOND_LIMIT_DEFAULT GATE_FPS_DEFAULT * 1280 * 800 * 2
#define PACKET_SIZE_DEFAULT 100000

void AutoCalibration::loggReport(const Report& report, unsigned int iteration) const {
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
            log("        {}    coverageAcquired    {:.1f}    dataAcquired    {:.1f}", i, coverageData.first, coverageData.second);
            i += 1;
        }
    } else {
        log("Recalibration  not triggered");
    }
    log("=================================================");
}

void AutoCalibration::loggConfig() const {
    auto log = [&](const std::string& fmt, auto&&... args) { logger->info(fmt, args...); };

    log("====== AutoCalibration Configuration ======");
    log("Mode:                   {}", initialConfig->mode == AutoCalibrationConfig::CONTINUOUS ? "CONTINUOUS" : "ON_START");
    log("Sleeping Time:          {}s", initialConfig->sleepingTime);
    log("Calib. Confidence Thr:  {:.2f}", initialConfig->calibrationConfidenceThreshold);
    log("Data Confidence Thr:    {:.2f}", initialConfig->dataConfidenceThreshold);
    log("Max Iterations:         {}", initialConfig->maxIterations);
    log("Max Images/Recalib:     {}", initialConfig->maxImagesPerReacalibration);
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
    auto numRawPool = camera->getRawNumFramesPool();
    auto numIspPool = camera->getIspNumFramesPool();
    auto numOutputsPool = camera->getOutputsNumFramesPool();
    if(numOutputsPool) {
        camera->setNumFramesPools(numRawPool + additionalPools, numIspPool + additionalPools, numOutputsPool.value() + additionalPools);
    } else {
        camera->setNumFramesPools(numRawPool + additionalPools, numIspPool + additionalPools, 6);
    }
}

std::shared_ptr<AutoCalibration> AutoCalibration::build(const std::shared_ptr<Camera>& cameraLeft, const std::shared_ptr<Camera>& cameraRight) {
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
    dynamicCalibration->syncInput.setMaxSize(std::max(initialConfig->validationSetSize, 2));
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
}

std::shared_ptr<dai::CalibrationMetrics> AutoCalibration::getMetrics(std::shared_ptr<dai::CalibrationHandler> calibration) {
    dynamicCalibrationCommandQueue.send(DCC::computeCalibrationMetrics(*calibration));
    return metricsQueue.get<dai::CalibrationMetrics>();
}

void AutoCalibration::buildInternal() {
    logger = pimpl->logger;
}

/**
return calibration from DynamicCalibration node
**/
std::shared_ptr<dai::CalibrationHandler> AutoCalibration::getNewCalibration(unsigned int maxNumIteration, Report& report) {
    auto startTime = std::chrono::steady_clock::now();  // Start timer
    gateControlQueue.send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    std::shared_ptr<CoverageData> coverage;
    for(unsigned int i = 0; i < maxNumIteration; i++) {
        dynamicCalibrationCommandQueue.send(DCC::startCalibration());
        for(unsigned int numLoadedImages = 0; numLoadedImages < initialConfig->maxImagesPerReacalibration; numLoadedImages++) {
            if(!mainLoop()) return nullptr;
            auto dynCalibrationResult = dynamicCalibrationQueue.get<dai::DynamicCalibrationResult>();
            coverage = coverageQueue.get<dai::CoverageData>();
            if(dynCalibrationResult->calibrationData) {
                if(dynCalibrationResult->calibrationData.value().dataConfidence > initialConfig->dataConfidenceThreshold) {
                    gateControlQueue.send(dai::GateControl::closeGate());
                    report.numIterationPerRecalibration = i + 1;
                    report.dataQualityAfterRecalibration = dynCalibrationResult->calibrationData.value().dataConfidence;
                    report.recalibrationPassed = true;
                    report.rotationDifference = dynCalibrationResult->calibrationData.value().calibrationDifference.rotationChange;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedRecalibrationSeconds = elapsed.count();
                    report.coveragesAcquired.push_back({coverage->coverageAcquired, coverage->dataAcquired});
                    return std::make_shared<dai::CalibrationHandler>(dynCalibrationResult->calibrationData.value().newCalibration);
                } else {
                    dynamicCalibrationCommandQueue.send(DCC::resetData());
                }
            }
        }
        if(coverage) {
            report.coveragesAcquired.push_back({coverage->coverageAcquired, coverage->dataAcquired});
        } else {
            report.coveragesAcquired.push_back({0., 0.});
        }
    }
    report.recalibrationPassed = false;
    report.numIterationPerRecalibration = maxNumIteration;
    gateControlQueue.send(dai::GateControl::closeGate());
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    report.elapsedRecalibrationSeconds = elapsed.count();
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
    unsigned int numIterations = 0;
    while(numIterations <= initialConfig->maxIterations && mainLoop()) {
        auto startTime = std::chrono::steady_clock::now();  // Start timer
        Report report;
        dynamicCalibrationCommandQueue.send(DCC::resetData());
        if(initialConfig->validationSetSize == 0) {
            auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT, report);
            if(newCalibration) {
                dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*newCalibration, initialConfig->flashCalibration));
                auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., true, *newCalibration);
                output.send(resultOutput);
                report.calibrationUpdated = true;
                auto endTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = endTime - startTime;
                report.elapsedSeconds = elapsed.count();  // Set duration
                loggReport(report, numIterations);
                return true;
            }
        } else {
            loadData(initialConfig->validationSetSize);
            auto metrics = getMetrics(calibration);
            report.dataConfidence = metrics->dataConfidence;
            report.calibrationConfidence = metrics->calibrationConfidence;
            if(metrics->dataConfidence > initialConfig->dataConfidenceThreshold) {
                if(metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold) {
                    dynamicCalibrationCommandQueue.send(DCC::applyCalibration(*calibration, initialConfig->flashCalibration));
                    auto resultOutput = std::make_shared<AutoCalibrationResult>(metrics->dataConfidence, metrics->calibrationConfidence, true, *calibration);
                    output.send(resultOutput);
                    report.calibrationUpdated = true;
                    auto endTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = endTime - startTime;
                    report.elapsedSeconds = elapsed.count();  // Set duration
                    loggReport(report, numIterations);
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
        loggReport(report, numIterations);
        ++numIterations;
    }

    if(mainLoop() && calibration) {
        auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., false, *calibration);
        output.send(resultOutput);
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
}

bool AutoCalibration::validateIncomingData() {
    std::chrono::seconds waitingTime(1);
    bool timedout = true;

    while(mainLoop()) {
        gateControlQueue.send(dai::GateControl::openGate(1, gate->initialConfig->fps));
        auto messageGroup = gateOutput.get<MessageGroup>(waitingTime, timedout);

        if(!timedout && messageGroup) {
            if(!messageGroup->get<dai::ImgFrame>(leftInputName) || !messageGroup->get<dai::ImgFrame>(rightInputName)) {
                logger->warn("Autocalibratino: Not initialized - Empty message groups.");
                return false;
            }

            auto leftImgFrame = messageGroup->get<ImgFrame>(leftInputName);
            auto rightImgFrame = messageGroup->get<ImgFrame>(rightInputName);

            if(leftImgFrame->getWidth() != rightImgFrame->getWidth() || leftImgFrame->getHeight() != rightImgFrame->getHeight()) {
                logger->warn("Autocalibratino: Not initialized - currently supports only sensors with same resolutions.");
                return false;
            }
            if(leftImgFrame->getHeight() != 800 || leftImgFrame->getWidth() != 1280) {
                logger->warn("Autocalibratino: Not initialized - currently supports only sensors with 1280x800 resolution.");
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
    loggConfig();
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
