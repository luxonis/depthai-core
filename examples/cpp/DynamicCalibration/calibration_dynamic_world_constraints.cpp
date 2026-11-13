// Dynamic calibration with a metric reference from markers in the scene.
//
// Image features fix the relative pose of the stereo pair except for its vergence, the turn about the axis
// perpendicular to the baseline and the optical axis, which sets the depth scale while barely changing the
// epipolar error. Known distances between marker centers (or from a marker center to a camera socket) and known
// marker sizes fix that turn; camera centers and the baseline length are never changed.
// Place the markers from the JSON file (ArUco by default; world_constraints_luxonis.json uses Luxonis Markers, the
// Luxonis lens logo printed as a concentric-ring fiducial with ids 0-5) in view of both cameras and run:
//
//     ./calibration_dynamic_world_constraints [--constraints world_constraints.json] [--host] [--apply]
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::filesystem::path constraintsPath = "world_constraints.json";
    bool runOnHost = false;
    bool apply = false;
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--host") {
            runOnHost = true;
        } else if(arg == "--apply") {
            apply = true;
        } else if(arg == "--constraints" && i + 1 < argc) {
            constraintsPath = argv[++i];
        } else {
            std::cerr << "Usage: " << argv[0] << " [--constraints world_constraints.json] [--host] [--apply]\n"
                      << "  --host   run the DynamicCalibration node on the host\n"
                      << "  --apply  apply the new calibration to the device (RAM only)\n";
            return 1;
        }
    }

    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    dynCalib->setWorldConstraintsFromFile(constraintsPath);
    if(runOnHost) {
        dynCalib->setRunOnHost(true);
    }
    left->requestFullResolutionOutput()->link(dynCalib->left);
    right->requestFullResolutionOutput()->link(dynCalib->right);

    auto calibrationQueue = dynCalib->calibrationOutput.createOutputQueue();
    auto coverageQueue = dynCalib->coverageOutput.createOutputQueue();
    auto control = dynCalib->inputControl.createInputQueue();

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    using DCC = dai::DynamicCalibrationControl;
    control->send(DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_PERFORMANCE));
    control->send(DCC::startCalibration());

    while(pipeline.isRunning()) {
        if(auto coverage = coverageQueue->tryGet<dai::CoverageData>()) {
            std::cout << std::fixed << std::setprecision(1) << "coverage " << coverage->meanCoverage << " %, data " << coverage->dataAcquired << " %\n";
        }
        auto result = calibrationQueue->tryGet<dai::DynamicCalibrationResult>();
        if(!result) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::cout << "Dynamic calibration status: " << result->info << '\n';
        if(result->calibrationData) {
            const auto& calibrationData = *result->calibrationData;
            for(const auto& [sockets, rotation] : calibrationData.calibrationDifference.pairwiseRotationDifference) {
                std::cout << std::setprecision(4) << "Rotation change " << sockets.first << "-" << sockets.second << ": (" << rotation[0] << ", " << rotation[1]
                          << ", " << rotation[2] << ") deg\n";
            }
            if(apply) {
                control->send(DCC::applyCalibration(calibrationData.newCalibration));
            }
            break;
        }
    }
    pipeline.stop();
    return 0;
}
