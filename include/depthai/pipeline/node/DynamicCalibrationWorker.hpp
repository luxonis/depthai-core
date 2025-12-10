#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationControl.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationResults.hpp>
#include <depthai/pipeline/node/DynamicCalibrationNode.hpp>
#include <depthai/properties/DynamicCalibrationWorkerProperties.hpp>

namespace dai {
namespace node {

class DynamicCalibrationWorker : public DeviceNodeCRTP<DeviceNode, DynamicCalibrationWorker, DynamicCalibrationWorkerProperties> {
   public:
    using DCC = dai::DynamicCalibrationControl;

    constexpr static const char* NAME = "DynamicCalibrationWorker";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    // synced left and right images
    Subnode<node::DynamicCalibration> dynamicCalibration{*this, "dynamicCalibration"};

    Input& syncedInput = dynamicCalibration->syncInput;

    void run() override;

   private:
    bool checkCalibration();

    void updateCalibration();

    void runContinuousMode();

    void runOnStartMode();

    bool isNewCalibrationOK(std::shared_ptr<dai::DynamicCalibrationResult> calibrationResult);

    void buildInternal() override;

    DynamicCalibrationWorkerProperties properties;

    std::shared_ptr<dai::InputQueue> dynamicCalibrationCommandQueue;
    std::shared_ptr<dai::MessageQueue> dynamicCalibrationCalibrationQueue;
};

}  // namespace node
}  // namespace dai
