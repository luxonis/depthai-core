#include "depthai/pipeline/node/DynamicCalibrationWorker.hpp"

#include "depthai/pipeline/InputQueue.hpp"

namespace dai {
namespace node {

void DynamicCalibrationWorker::buildInternal() {
    // TODO check left right inputs
    auto inputQueue = syncedInput.createInputQueue();
    dynamicCalibrationCalibrationQueue = dynamicCalibration->calibrationOutput.createOutputQueue();
    dynamicCalibrationCommandQueue = dynamicCalibration->inputControl.createInputQueue();
}

bool DynamicCalibrationWorker::isNewCalibrationOK(std::shared_ptr<dai::DynamicCalibrationResult> calibrationResult) {
    if(!calibrationResult) {
        return false;
    }
    if(!calibrationResult->calibrationData) {
        return false;
    }
    // TODO IMPLEMENT THE LOGIC
    // validate on a subsets? here or in DCL?
    // check Sampson error ?
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
        auto dynCalibrationResult = dynamicCalibrationCalibrationQueue->get<dai::DynamicCalibrationResult>();

        // Check that the incomming calibration is OK
        if(isNewCalibrationOK(dynCalibrationResult)) {
            succesfullyRecalibrated = true;
            dynamicCalibrationCommandQueue->send(DCC::applyCalibration(dynCalibrationResult->calibrationData->newCalibration));
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
        std::this_thread::sleep_for(std::chrono::seconds(properties.sleepingTime));
    }
}

void DynamicCalibrationWorker::runOnStartMode() {
    auto isCalibrationOK = checkCalibration();
    if(!isCalibrationOK) {
        updateCalibration();
    }
}

void DynamicCalibrationWorker::run() {
    switch(properties.mode) {
        case DynamicCalibrationWorkerProperties::Mode::CONTINUOUS:
            runContinuousMode();
            break;
        case DynamicCalibrationWorkerProperties::Mode::ON_START:
            runOnStartMode();
            // properly destroy the Node ??
            break;
    }
}

}  // namespace node
}  // namespace dai
