#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("Test manual focus") {
    dai::Pipeline pipeline;
    auto fps = 30.0f;
    const auto device = pipeline.getDefaultDevice();
    auto features = device->getConnectedCameraFeatures();
    std::cout << "Connected Camera Features:\n" << features << "\n" << std::flush;
    // Find CAM_A and check if it has autofocus
    auto it = std::find_if(features.begin(), features.end(), [](const dai::CameraFeatures& f) { return f.socket == dai::CameraBoardSocket::CAM_A; });

    if(it == features.end()) {
        std::cout << "CAM_A not found. Skipping test.\n";
        return;  // Skip the test
    }

    if(!it->hasAutofocus) {
        std::cout << "CAM_A does not support autofocus. Skipping test.\n";
        return;  // Skip the test
    }
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* output = camRgb->requestOutput(std::make_pair(640, 480), std::nullopt, dai::ImgResizeMode::CROP, fps);
    auto outputQueue = output->createOutputQueue();
    auto controlQueue = camRgb->inputControl.createInputQueue();
    pipeline.start();
    for(int idx = 0; idx < static_cast<int>(fps); ++idx) {
        (void)outputQueue->get<dai::ImgFrame>();
    }
    for(auto requestedLensPos : {0.5f, 0.8f, 1.0f, 0.733f, 0.444f, 0.2101f, 0.0f}) {
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualFocusRaw(requestedLensPos);
        std::cout << "Setting lens position to " << requestedLensPos << "\n" << std::flush;
        controlQueue->send(ctrl);
        auto lastLensPosition = -1.0f;
        for(int idx = 0; idx < static_cast<int>(fps); ++idx) {
            auto imgFrame = outputQueue->get<dai::ImgFrame>();
            lastLensPosition = imgFrame->getLensPositionRaw();
        }
        std::cout << "Lens position after 1s was: " << lastLensPosition << "\n" << std::flush;
        auto lensPositionDiff = std::abs(lastLensPosition - requestedLensPos);
        std::cout << "Diff was: " << lensPositionDiff << "\n" << std::flush;
        REQUIRE(lensPositionDiff < 0.001);
    }
    pipeline.stop();
}

TEST_CASE("Test manual focus initial config") {
    dai::Pipeline pipeline;
    auto fps = 30.0f;
    const auto device = pipeline.getDefaultDevice();
    auto features = device->getConnectedCameraFeatures();
    std::cout << features << "Connected Camera Features:\n";

    // Find CAM_A and check if it has autofocus
    auto it = std::find_if(features.begin(), features.end(), [](const dai::CameraFeatures& f) { return f.socket == dai::CameraBoardSocket::CAM_A; });

    if(it == features.end()) {
        std::cout << "CAM_A not found. Skipping test.\n";
        return;  // Skip the test
    }

    if(!it->hasAutofocus) {
        std::cout << "CAM_A does not support autofocus. Skipping test.\n";
        return;  // Skip the test
    }

    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* output = camRgb->requestOutput(std::make_pair(640, 480), std::nullopt, dai::ImgResizeMode::CROP, fps);
    auto requestedLensPos = 0.643f;
    std::cout << "Setting initial lens position to " << requestedLensPos << "\n" << std::flush;
    auto ctrl = std::make_shared<dai::CameraControl>();
    camRgb->initialControl.setManualFocusRaw(requestedLensPos);
    auto outputQueue = output->createOutputQueue();
    pipeline.start();
    auto lastLensPosition = -1.0f;
    for(int idx = 0; idx < static_cast<int>(fps); ++idx) {
        auto imgFrame = outputQueue->get<dai::ImgFrame>();
        lastLensPosition = imgFrame->getLensPositionRaw();
    }
    std::cout << "Lens position after 1s was: " << lastLensPosition << "\n" << std::flush;
    auto lensPositionDiff = std::abs(lastLensPosition - requestedLensPos);
    std::cout << "Diff was: " << lensPositionDiff << "\n" << std::flush;
    REQUIRE(lensPositionDiff < 0.001);
    pipeline.stop();
}
