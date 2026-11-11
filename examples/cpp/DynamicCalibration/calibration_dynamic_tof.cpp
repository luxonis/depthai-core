#include <algorithm>
#include <argparse/argparse.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <stdexcept>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    argparse::ArgumentParser program("calibration_dynamic_tof");
    program.add_description("Native ToF depth/amplitude extrinsic refinement against CAM_B");
    program.add_argument("--device").default_value(std::string()).help("Device IP or ID; defaults to the first available device");
    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& error) {
        std::cerr << error.what() << '\n' << program;
        return 1;
    }
    const auto deviceId = program.get<std::string>("--device");

    using DCC = dai::DynamicCalibrationControl;
    auto device = deviceId.empty() ? std::make_shared<dai::Device>() : std::make_shared<dai::Device>(dai::DeviceInfo(deviceId));
    dai::Pipeline pipeline(device);
    pipeline.setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    constexpr float FPS = 15.0f;
    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, FPS);
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto* leftOut = left->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, FPS, false);

    auto dyn = pipeline.create<dai::node::DynamicCalibration>();
    dyn->setRunOnHost(true);
    dyn->sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<float>(0.5f / FPS)));
    // Depth is an auxiliary measurement from the ToF sensor.
    tof->amplitude.link(dyn->inputs["tof"]);
    leftOut->link(dyn->left);
    tof->depth.link(dyn->tofDepth);

    const std::map<std::string, std::shared_ptr<dai::MessageQueue>> previews = {
        {"ToF amplitude", tof->amplitude.createOutputQueue(1, false)},
        {"CAM_B", leftOut->createOutputQueue(1, false)},
    };
    auto coverageQueue = dyn->coverageOutput.createOutputQueue(4, false);
    auto calibrationQueue = dyn->calibrationOutput.createOutputQueue(4, false);
    auto control = dyn->inputControl.createInputQueue();

    pipeline.start();
    control->send(DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_SPEED));
    control->send(DCC::startCalibration());
    std::optional<dai::CalibrationHandler> candidate;
    std::cout << "Native metric ToF edge refinement on host\n";
    std::cout << "Collect common edges at several depths and orientations. Keys: a=apply candidate in memory, r=collect again, q=quit." << std::endl;

    while(pipeline.isRunning()) {
        for(const auto& [name, queue] : previews) {
            if(auto frame = queue->tryGet<dai::ImgFrame>()) {
                auto image = frame->getCvFrame();
                if(name == "ToF amplitude") {
                    // Preview only; the node scales native amplitude separately.
                    cv::Mat samples;
                    image.reshape(1, 1).convertTo(samples, CV_32F);
                    auto* values = samples.ptr<float>();
                    auto* percentile = values + (samples.total() - 1) * 99 / 100;
                    std::nth_element(values, percentile, values + samples.total());
                    cv::Mat preview;
                    image.convertTo(preview, CV_8U, 255. / std::max(1.f, *percentile));
                    image = preview;
                }
                cv::imshow(name, image);
            }
        }

        if(auto coverage = coverageQueue->tryGet<dai::CoverageData>()) {
            std::cout << "Collection progress: " << coverage->dataAcquired << '%' << std::endl;
        }
        if(auto result = calibrationQueue->tryGet<dai::DynamicCalibrationResult>()) {
            std::cout << "Calibration: " << result->info << std::endl;
            if(result->calibrationData) {
                candidate = result->calibrationData->newCalibration;
                const auto& quality = result->calibrationData->calibrationDifference;
                std::cout << "Sampson error: " << quality.sampsonErrorCurrent << " -> " << quality.sampsonErrorNew << " px\n";
                for(const auto& [pair, rotation] : quality.pairwiseRotationDifference) {
                    std::cout << "Rotation change " << dai::toString(pair.first) << " -> " << dai::toString(pair.second) << ": ";
                    for(const auto value : rotation) std::cout << value << ' ';
                    std::cout << "deg\n";
                }
                std::cout << "Candidate ready. Press a to apply it in memory or r to collect a different view." << std::endl;
            }
        }

        const auto key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == 'a' && candidate) {
            control->send(DCC::applyCalibration(*candidate, false));
            candidate.reset();
            std::cout << "Requested runtime calibration update; EEPROM unchanged." << std::endl;
        }
        if(key == 'r') {
            candidate.reset();
            control->send(DCC::resetData());
            control->send(DCC::startCalibration());
        }
    }

    pipeline.stop();
    cv::destroyAllWindows();
    return 0;
}
