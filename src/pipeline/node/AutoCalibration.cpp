#include "depthai/pipeline/node/AutoCalibration.hpp"

#include <pipeline/ThreadedNodeImpl.hpp>
#include <pipeline/datatype/MessageGroup.hpp>

#include "depthai/pipeline/InputQueue.hpp"

namespace dai {
namespace node {

AutoCalibration::~AutoCalibration() = default;

AutoCalibration::Properties& AutoCalibration::getProperties() {
    return properties;
}

void AutoCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

std::shared_ptr<AutoCalibration> AutoCalibration::build(const std::shared_ptr<Camera> cameraLeft, const std::shared_ptr<Camera> cameraRight) {
    sync->setRunOnHost(false);
    gate->setRunOnHost(false);
    auto outputCameraLeft = cameraLeft->requestIspOutput();
    auto outputCameraRight = cameraRight->requestIspOutput();
    outputCameraLeft->link(left);
    outputCameraRight->link(right);
    sync->out.link(gate->input);
    gate->output.link(dynamicCalibration->syncInput);
    return std::static_pointer_cast<AutoCalibration>(shared_from_this());
}

/**
 * Check if the node is set to run on host
 */
bool AutoCalibration::runOnHost() const {
    return runOnHostVar;
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
    gate->initialConfig->fps = 5;
    dynamicCalibration->syncInput.setMaxSize(std::max(initialConfig->validationSetSize, 2));
}

void AutoCalibration::buildInternal() {
    logger = pimpl->logger;
}

std::shared_ptr<dai::CalibrationHandler> AutoCalibration::getNewCalibration(unsigned int maxNumIteration) {
    gateControlQueue->send(dai::GateControl::openGate(-1, gate->initialConfig->fps));
    for(unsigned int i = 0; i < maxNumIteration; i++) {
        dynamicCalibrationCommandQueue->send(DCC::startCalibration());
        for(unsigned int numLoadedImages = 0; numLoadedImages < initialConfig->maxImagesPerReacalibration; numLoadedImages++) {
            auto dynCalibrationResult = dynamicCalibrationQueue->get<dai::DynamicCalibrationResult>();
            coverageQueue->tryGet<dai::CoverageData>();
            if(dynCalibrationResult->calibrationData) {
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

bool AutoCalibration::recalibrate(std::shared_ptr<dai::CalibrationHandler> calibration) {
    unsigned int numIterations = 0;
    while(numIterations <= initialConfig->maxIterations && isRunning()) {
        dynamicCalibrationCommandQueue->send(DCC::resetData());
        if(initialConfig->validationSetSize == 0) {
            auto newCalibration = getNewCalibration(5);
            if(newCalibration) {
                dynamicCalibrationCommandQueue->send(DCC::applyCalibration(*newCalibration, initialConfig->flashCalibration));
                auto resultOutput = std::make_shared<AutoCalibrationResult>(0., 0., true, *newCalibration);
                output.send(resultOutput);
                return true;
            }
        } else {
            loadData(initialConfig->validationSetSize);
            auto metrics = getMetrics(calibration);
            if(metrics->dataConfidence > initialConfig->dataConfidenceThreshold) {
                if(metrics->calibrationConfidence > initialConfig->calibrationConfidenceThreshold) {
                    dynamicCalibrationCommandQueue->send(DCC::applyCalibration(*calibration, initialConfig->flashCalibration));
                    auto resultOutput = std::make_shared<AutoCalibrationResult>(metrics->dataConfidence, metrics->calibrationConfidence, true, *calibration);
                    output.send(resultOutput);
                    return true;
                }
                auto newCalibration = getNewCalibration(5);
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
    return false;
}

void AutoCalibration::runContinuousMode() {
    while(isRunning()) {
        recalibrate(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
        std::this_thread::sleep_for(std::chrono::seconds(initialConfig->sleepingTime));
    }
}

void AutoCalibration::runOnStartMode() {
    recalibrate(std::make_shared<dai::CalibrationHandler>(device->getCalibration()));
}

bool AutoCalibration::validateIncomingData() {
    std::chrono::seconds waitingTime(1);
    bool timedout = true;

    while(isRunning()) {
        gateControlQueue->send(dai::GateControl::openGate(1, gate->initialConfig->fps));
        auto messageGroup = gateOutput->get<MessageGroup>(waitingTime, timedout);

        if(!timedout && messageGroup) {
            if(!messageGroup->get<dai::ImgFrame>(leftInputName) || !messageGroup->get<dai::ImgFrame>(rightInputName)) {
                return false;
            }

            auto leftImgFrame = messageGroup->get<ImgFrame>(leftInputName);
            auto rightImgFrame = messageGroup->get<ImgFrame>(rightInputName);

            if(leftImgFrame->getWidth() != rightImgFrame->getWidth() || leftImgFrame->getHeight() != rightImgFrame->getHeight()) {
                return false;
            }
            if(leftImgFrame->getHeight() != 800 || leftImgFrame->getWidth() != 1280) {
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
        logger->info("Invalid incoming data autocalibration stopped!");
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
    }
}

}  // namespace node
}  // namespace dai
