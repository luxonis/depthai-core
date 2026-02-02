#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Gate.hpp"

TEST_CASE("Test Gate Timing and Data Flow") {
    dai::Pipeline pipeline;

    // 1. Setup Nodes
    auto camera = pipeline.create<dai::node::Camera>()->build();
    // We set 30fps: 2 seconds should yield ~60 frames
    auto cameraOut = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);

    auto gate = pipeline.create<dai::node::Gate>();
    cameraOut->link(gate->input);

    // 2. Setup Queues
    auto cameraQueue = gate->output.createOutputQueue(8, false);  // Non-blocking
    auto gateControlQueue = gate->inputControl.createInputQueue();

    pipeline.start();

    // Test Parameters
    const int numCycles = 4;            // Number of times to toggle
    const double periodDuration = 2.0;  // 2 seconds per state
    const double waitDelay = 0.05;      // 0.05s stabilization wait
    bool currentGateState = true;       // Start with Gate ON as requested

    auto startTime = std::chrono::steady_clock::now();
    int cycleCount = 0;
    int framesInPeriod = 0;
    bool isMeasuring = false;

    // Initial Gate Control (On)
    auto ctrl = std::make_shared<dai::GateControl>(currentGateState, -1);
    gateControlQueue->send(ctrl);

    std::cout << "--- Starting Test: Gate is ON ---" << std::endl;

    while(pipeline.isRunning() && cycleCount < numCycles) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - startTime;

        if(elapsed.count() >= periodDuration) {
            std::cout << "\n--- Cycle " << cycleCount << ": Toggling Gate to " << (currentGateState ? "ON" : "OFF") << " ---" << std::endl;
            if(currentGateState) {
                CHECK(framesInPeriod > 30);
                std::cout << "[VERIFY] Gate ON period finished. Frames received: " << framesInPeriod << std::endl;
            } else {
                CHECK(framesInPeriod == 0);
                std::cout << "[VERIFY] Gate OFF period finished. Frames received: " << framesInPeriod << std::endl;
            }

            // Toggle State
            currentGateState = !currentGateState;
            ctrl->open = currentGateState;
            gateControlQueue->send(ctrl);

            // Reset Period
            startTime = now;
            framesInPeriod = 0;
            isMeasuring = false;
            cycleCount++;
            continue;
        }
        auto frame = cameraQueue->tryGet<dai::ImgFrame>();

        if(elapsed.count() >= waitDelay) {
            isMeasuring = true;
        }

        if(isMeasuring) {
            if(frame) {
                framesInPeriod++;
            }
        }
    }
}

TEST_CASE("Test Gate N Messages") {
    dai::Pipeline pipeline;

    // 1. Setup Nodes
    auto camera = pipeline.create<dai::node::Camera>()->build();
    // We set 30fps: 2 seconds should yield ~60 frames
    auto cameraOut = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);

    auto gate = pipeline.create<dai::node::Gate>();
    cameraOut->link(gate->input);

    // 2. Setup Queues
    auto cameraQueue = gate->output.createOutputQueue(8, false);  // Non-blocking
    auto gateControlQueue = gate->inputControl.createInputQueue();

    // Test Parameters
    const int numMessages = 8;
    const int numCycles = 4;            // Number of times to toggle
    const double periodDuration = 2.0;  // 2 seconds per state
    const double waitDelay = 0.05;      // 0.05s stabilization wait

    auto startTime = std::chrono::steady_clock::now();
    int cycleCount = 0;
    int framesInPeriod = 0;

    // Initial Gate Control (On)
    auto ctrl = std::make_shared<dai::GateControl>(true, numMessages);
    gate->initialConfig->open = true;
    gate->initialConfig->numMessages = numMessages;

    pipeline.start();

    std::cout << "--- Starting Test: Gate is ON ---" << std::endl;

    while(pipeline.isRunning() && cycleCount < numCycles) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - startTime;

        if(elapsed.count() >= periodDuration) {
            CHECK(framesInPeriod == numMessages);
            gateControlQueue->send(ctrl);
            std::cout << "[VERIFY] Gate ON for N messages period finished. Frames received: " << framesInPeriod << std::endl;

            // Reset Period
            startTime = now;
            framesInPeriod = 0;
            cycleCount++;
            continue;
        }
        auto frame = cameraQueue->tryGet<dai::ImgFrame>();

        if(frame) {
            framesInPeriod++;
        }
    }
}
