#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <DynamicCalibration.hpp>
#include <CalibrationHandle.hpp>
#include <SensorHandle.hpp>

namespace dai {
namespace node {

enum class CalibrationState {
    Idle,
    InitializingPipeline,
    CollectingFeatures,
    ProcessingQuality,
    Recalibrating
};

enum class CalibrationMode {
    None,
    QualityCheck,
    Recalibration
};

// Then stringifier
inline const char* toString(CalibrationState state) {
    switch(state) {
        case CalibrationState::Idle: return "Idle";
        case CalibrationState::InitializingPipeline: return "InitializingPipeline";
        case CalibrationState::CollectingFeatures: return "CollectingFeatures";
        case CalibrationState::ProcessingQuality: return "ProcessingQuality";
        case CalibrationState::Recalibrating: return "Recalibrating";
        default: return "Unknown";
    }
}


struct CalibrationResult {
    std::optional<CalibrationHandler> calibration;
    bool valid = false;
    std::string info;

    static CalibrationResult Invalid(std::string reason = "No result") {
        return CalibrationResult{std::nullopt, false, std::move(reason)};
    }
};

struct QualityResult {
    double value = -1.0f;
    bool valid = false;
    std::string info;

    static QualityResult Invalid(std::string reason = "No result") {
        return QualityResult{-1.0f, false, std::move(reason)};
    }
};

// Then dependent structs
struct CalibrationResults {
    QualityResult quality;
    CalibrationResult calibration;

    void reset() {
        quality = QualityResult::Invalid();
        calibration = CalibrationResult::Invalid();
    }
};

struct CalibrationStateMachine {
    CalibrationState state = CalibrationState::Idle;
    CalibrationMode mode = CalibrationMode::None;
    int collectedFrames = 0;
    bool pipelineReady = false;

    void startQualityCheck() {
        if(isIdle()) {
            state = CalibrationState::InitializingPipeline;
            mode = CalibrationMode::QualityCheck;
            collectedFrames = 0;
            pipelineReady = false;
        }
    }

    void startRecalibration() {
        if(isIdle()) {
            state = CalibrationState::InitializingPipeline;
            mode = CalibrationMode::Recalibration;
            collectedFrames = 0;
            pipelineReady = false;
        }
    }

    void markPipelineReady() {
        pipelineReady = true;
        state = CalibrationState::CollectingFeatures;
    }

    bool isIdle() const {
        return state == CalibrationState::Idle;
    }

    void maybeAdvanceAfterCollection() {  // TODO, REPLACE WITH ANYTHING MEANINGFUL FROM THE DCL ITSELF
        if(mode == CalibrationMode::QualityCheck && collectedFrames >= 3) { 
            state = CalibrationState::ProcessingQuality;
        } else if(mode == CalibrationMode::Recalibration && collectedFrames >= 10) {
            state = CalibrationState::Recalibrating;
        }
    }

    void finish() {
        state = CalibrationState::Idle;
        mode = CalibrationMode::None;
        collectedFrames = 0;
        pipelineReady = false;
    }

    std::string stateToString() const {
        return std::string(toString(state));
    }
};

/**
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   private:
    bool runOnHostVar = true;

   public:
    constexpr static const char* NAME = "DynamicCalibration";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    CalibrationResults results;

    // We could have a map here to perform the calibration on an arbitrary number of inputs for example: (left, right, rgb).
    // /**
    //  * A map of inputs
    //  */
    // InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Input left{*this, {"left", DEFAULT_GROUP, false, 1}};
    Input right{*this, {"right", DEFAULT_GROUP, false, 1}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Get the calibration quality (epipolar error)
     * @return Epipolar error in pixels
     */

    std::unique_ptr<dcl::CameraSensorHandle> createDCLCameraCalibration(const std::vector<std::vector<float>> cameraMatrix, 
        const std::vector<float> distortionCoefficients, 
        const std::vector<std::vector<float>> rotationMatrix,   
        const std::vector<float> translationVector,
        int widthDefault, 
        int heightDefault);

    void startCalibQualityCheck();
    void startRecalibration();
    QualityResult getCalibQuality() const;
    CalibrationResult getNewCalibration() const;

    void pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket, int widthDefault = 1280, int heightDefault = 800);
    void run() override;

    private:
        std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
        std::unique_ptr<dcl::DynamicCalibration> dynCalibImpl;
        std::shared_ptr<dcl::Device> dcDevice;
        dcl::mxid_t deviceName;
        dcl::socket_t socketA;
        dcl::socket_t socketB;

        CalibrationStateMachine calibrationSM;
};

}  // namespace node
}  // namespace dai
