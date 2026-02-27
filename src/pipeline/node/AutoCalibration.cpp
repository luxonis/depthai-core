#include "depthai/pipeline/node/AutoCalibration.hpp"

#include <pipeline/ThreadedNodeImpl.hpp>
#include <pipeline/datatype/MessageGroup.hpp>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"

namespace dai {
namespace node {

#define MAX_FAILS_PER_RECALIBRATION_DEFAULT 5
#define GATE_FPS_DEFAULT 5
#define BYTES_PES_SECOND_LIMIT_DEFAULT 5000000
#define PACKET_SIZE_DEFAULT 100000

AutoCalibration::~AutoCalibration() = default;

AutoCalibration::Properties& AutoCalibration::getProperties() {
    return properties;
}

void AutoCalibration::updateStatus(AutoCalibrationExecutionStatus newStatus) {
    std::lock_guard<std::mutex> lock(statusMtx);
    status.status = newStatus;
}

void AutoCalibration::updateResult(double dataConfidence, double calibrationConfidence, bool calibrationResult) {
    std::lock_guard<std::mutex> lock(statusMtx);
    status.dataConfidence = dataConfidence;
    status.calibrationConfidence = calibrationConfidence;
    status.calibrationResult = calibrationResult;
}

AutoCalibrationStatus AutoCalibration::getAutoCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(statusMtx);
    return status;
}

void AutoCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
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
    cameraLeft->setNumFramesPools(6, 6, 6);
    cameraRight->setNumFramesPools(6, 6, 6);
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
    xlinkBridge->xLinkOut->setBytesPerSecondLimit(BYTES_PES_SECOND_LIMIT_DEFAULT);
    xlinkBridge->xLinkOut->setPacketSize(PACKET_SIZE_DEFAULT);
    xlinkBridge->xLinkOut->input.setMaxSize(1);
    xlinkBridge->xLinkOut->input.setBlocking(false);
}

void AutoCalibration::loadData(unsigned int numImages) {
    coverageQueue->tryGetAll<dai::CoverageData>();
    gateControlQueue->send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    for(unsigned int i = 0; i < numImages; i++) {
        dynamicCalibrationCommandQueue->send(DCC::loadImage());
        coverageQueue->get<dai::CoverageData>();  // wait until the data are loaded
    }
    gateControlQueue->send(dai::GateControl::closeGate());
}

std::shared_ptr<dai::CalibrationMetrics> AutoCalibration::getMetrics(std::shared_ptr<dai::CalibrationHandler> calibration) {
    dynamicCalibrationCommandQueue->send(DCC::computeCalibrationMetrics(*calibration));
    return metricsQueue->get<dai::CalibrationMetrics>();
}

void AutoCalibration::buildStage1() {
    dynamicCalibrationQueue = dynamicCalibration->calibrationOutput.createOutputQueue();
    coverageQueue = dynamicCalibration->coverageOutput.createOutputQueue();
    metricsQueue = dynamicCalibration->metricsOutput.createOutputQueue();
    dynamicCalibrationCommandQueue = dynamicCalibration->inputControl.createInputQueue();
    gateControlQueue = gate->inputControl.createInputQueue();
    gateOutput = gate->output.createOutputQueue();
    gate->initialConfig->open = false;
    gate->initialConfig->fps = GATE_FPS_DEFAULT;
    dynamicCalibration->syncInput.setMaxSize(std::max(initialConfig->validationSetSize, 2));
}

void AutoCalibration::buildInternal() {
    logger = pimpl->logger;
}

/**
return calibration from DynamicCalibration node
**/
std::shared_ptr<dai::CalibrationHandler> AutoCalibration::getNewCalibration(unsigned int maxNumIteration) {
    gateControlQueue->send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    for(unsigned int i = 0; i < maxNumIteration; i++) {
        dynamicCalibrationCommandQueue->send(DCC::startCalibration());
        for(unsigned int numLoadedImages = 0; numLoadedImages < initialConfig->maxImagesPerReacalibration; numLoadedImages++) {
            auto dynCalibrationResult = dynamicCalibrationQueue->get<dai::DynamicCalibrationResult>();
            coverageQueue->tryGet<dai::CoverageData>();
            if(dynCalibrationResult->calibrationData) {
                logger->warn("dynCalibrationResult->calibrationData.value().dataConfidence {}", dynCalibrationResult->calibrationData.value().dataConfidence);
                if(dynCalibrationResult->calibrationData.value().dataConfidence > initialConfig->dataConfidenceThreshold) {
                    gateControlQueue->send(dai::GateControl::closeGate());
                    return std::make_shared<dai::CalibrationHandler>(dynCalibrationResult->calibrationData.value().newCalibration);
                }
            }
        }
        dynamicCalibrationCommandQueue->send(DCC::resetData());
    }
    gateControlQueue->send(dai::GateControl::closeGate());
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
    updateStatus(AutoCalibrationExecutionStatus::CALIBRATING);
    unsigned int numIterations = 0;
    while(numIterations <= initialConfig->maxIterations && isRunning()) {
        dynamicCalibrationCommandQueue->send(DCC::resetData());
        if(initialConfig->validationSetSize == 0) {
            auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT);
            if(newCalibration) {
                dynamicCalibrationCommandQueue->send(DCC::applyCalibration(*newCalibration, initialConfig->flashCalibration));
                auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., true, *newCalibration);
                output.send(resultOutput);
                updateResult(0.0, 0.0, true);
                updateStatus(AutoCalibrationExecutionStatus::SUCCEEDED);
                return true;
            }
        } else {
            loadData(initialConfig->validationSetSize);
            auto metrics = getMetrics(calibration);
            logger->warn("metrics->dataConfidence {}", metrics->dataConfidence);
            logger->warn("initialConfig->dataConfidenceThreshold {}", initialConfig->dataConfidenceThreshold);
            logger->warn("metrics->calibrationConfidence {}", metrics->calibrationConfidence);
            if(metrics->dataConfidence > initialConfig->dataConfidenceThreshold) {
                if(metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold) {
                    dynamicCalibrationCommandQueue->send(DCC::applyCalibration(*calibration, initialConfig->flashCalibration));
                    auto resultOutput = std::make_shared<AutoCalibrationResult>(metrics->dataConfidence, metrics->calibrationConfidence, true, *calibration);
                    output.send(resultOutput);
                    updateResult(metrics->dataConfidence, metrics->calibrationConfidence, true);
                    updateStatus(AutoCalibrationExecutionStatus::SUCCEEDED);
                    return true;
                }
                auto newCalibration = getNewCalibration(MAX_FAILS_PER_RECALIBRATION_DEFAULT);
                if(newCalibration) {
                    calibration = std::move(newCalibration);
                }
            }
        }
        ++numIterations;
    }

    if(isRunning() && calibration) {
        auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., false, *calibration);
        output.send(resultOutput);
    }
    updateResult(0.0, 0.0, false);
    updateStatus(AutoCalibrationExecutionStatus::FAILED);
    return false;
}

void AutoCalibration::runContinuousMode() {
    while(isRunning()) {
        updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
        std::this_thread::sleep_for(std::chrono::seconds(initialConfig->sleepingTime));
    }
}

void AutoCalibration::runOnStartMode() {
    updateCalibrationProcess(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
}

bool AutoCalibration::validateIncomingData() {
    updateStatus(AutoCalibrationExecutionStatus::VALIDATING_INPUT);
    std::chrono::seconds waitingTime(1);
    bool timedout = true;

    while(isRunning()) {
        gateControlQueue->send(dai::GateControl::openGate(1, gate->initialConfig->fps));
        auto messageGroup = gateOutput->get<MessageGroup>(waitingTime, timedout);

        if(timedout) {
            updateStatus(AutoCalibrationExecutionStatus::TIMEOUT);
            continue;
        }

        if(!timedout && messageGroup) {
            if(!messageGroup->get<dai::ImgFrame>(leftInputName) || !messageGroup->get<dai::ImgFrame>(rightInputName)) {
                updateStatus(AutoCalibrationExecutionStatus::INVALID_INPUT);
                return false;
            }

            auto leftImgFrame = messageGroup->get<ImgFrame>(leftInputName);
            auto rightImgFrame = messageGroup->get<ImgFrame>(rightInputName);

            if(leftImgFrame->getWidth() != rightImgFrame->getWidth() || leftImgFrame->getHeight() != rightImgFrame->getHeight()) {
                updateStatus(AutoCalibrationExecutionStatus::INVALID_INPUT);
                return false;
            }
            if(leftImgFrame->getHeight() != 800 || leftImgFrame->getWidth() != 1280) {
                updateStatus(AutoCalibrationExecutionStatus::INVALID_INPUT);
                return false;
            }
            updateStatus(AutoCalibrationExecutionStatus::RUNNING);
            return true;
        }
    }
    updateStatus(AutoCalibrationExecutionStatus::STOPPED);
    return false;
}

void AutoCalibration::run() {
    logger->info("AutoCalibration started to work!");
    updateStatus(AutoCalibrationExecutionStatus::RUNNING);
    if(!validateIncomingData()) {
        logger->info("Invalid incoming data autocalibration stopped!");
        updateStatus(AutoCalibrationExecutionStatus::INVALID_INPUT);
        return;
    }
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
    if(!isRunning()) {
        updateStatus(AutoCalibrationExecutionStatus::STOPPED);
    }
}

}  // namespace node
}  // namespace dai
