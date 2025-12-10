#include "depthai/pipeline/node/DynamicCalibrationWorker.hpp"

#include <pipeline/ThreadedNodeImpl.hpp>

#include "depthai/pipeline/InputQueue.hpp"

namespace dai {
namespace node {

DynamicCalibrationWorker::~DynamicCalibrationWorker() = default;

DynamicCalibrationWorker::Properties& DynamicCalibrationWorker::getProperties() {
    return properties;
}

void DynamicCalibrationWorker::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool DynamicCalibrationWorker::runOnHost() const {
    return runOnHostVar;
}

void DynamicCalibrationWorker::buildInternalQueues() {
    // TODO check left right inputs
    dynamicCalibrationCalibrationQueue = dynamicCalibration->calibrationOutput.createOutputQueue();
    dynamicCalibrationCommandQueue = dynamicCalibration->inputControl.createInputQueue();
}

void DynamicCalibrationWorker::buildInternal() {
    logger = pimpl->logger;
}

bool DynamicCalibrationWorker::isNewCalibrationOK(std::shared_ptr<dai::DynamicCalibrationResult> calibrationResult) {
    if(!calibrationResult) {
        return false;
    }
    if(!calibrationResult->calibrationData) {
        return false;
    }
    if(calibrationResult->calibrationData->calibrationDifference.sampsonErrorNew > initialConfig->sampsonErrorThreshold) {
        return false;
    }
    // TODO IMPLEMENT THE LOGIC
    // validate on a subsets? here or in DCL?
    // check fillrate ?
    return true;
}

// Check the current calibration
bool DynamicCalibrationWorker::checkCalibration() {
    // TODO IMPLEMENT THE LOGIC
    // Sampson error OK?
    // Fillrate OK?
    return false;
}

void DynamicCalibrationWorker::updateCalibration() {
    bool succesfullyRecalibrated = false;
    while(!succesfullyRecalibrated) {
        dynamicCalibrationCommandQueue->send(DCC::startCalibration());

        bool dataCollected = false;
        while(!dataCollected) {
            auto dynCalibrationResult = dynamicCalibrationCalibrationQueue->get<dai::DynamicCalibrationResult>();
            if(dynCalibrationResult->calibrationData) {
                dataCollected = true;
            }
            // Check that the incomming calibration is OK
            if(isNewCalibrationOK(dynCalibrationResult)) {
                succesfullyRecalibrated = true;
                dynamicCalibrationCommandQueue->send(DCC::applyCalibration(dynCalibrationResult->calibrationData->newCalibration));
                logger->info("Recalibrated");
            }
        }
        dynamicCalibrationCommandQueue->send(DCC::resetData());
    }
}

void DynamicCalibrationWorker::runContinuousMode() {
    while(true) {
        auto isCalibrationOK = checkCalibration();
        if(!isCalibrationOK) {
            updateCalibration();
        }
        // trigger also by a low fillrate?
        std::this_thread::sleep_for(std::chrono::seconds(initialConfig->sleepingTime));
    }
}

void DynamicCalibrationWorker::runOnStartMode() {
    auto isCalibrationOK = checkCalibration();
    if(!isCalibrationOK) {
        updateCalibration();
    }
}

void DynamicCalibrationWorker::run() {
    logger->info("DynamicCalibrationWorker started to work!");
    switch(initialConfig->mode) {
        case DynamicCalibrationWorkerConfig::Mode::CONTINUOUS:
            logger->info("Running continuous mode.");
            runContinuousMode();
            break;
        case DynamicCalibrationWorkerConfig::Mode::ON_START:
            logger->info("Running on-start mode.");
            runOnStartMode();
            logger->info("On-start mode finished.");
            // properly destroy the Node ??
            break;
    }
}

}  // namespace node
}  // namespace dai
