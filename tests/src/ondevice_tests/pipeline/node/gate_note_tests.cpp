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
    const int num_cycles = 4;            // Number of times to toggle
    const double period_duration = 2.0;  // 2 seconds per state
    const double wait_delay = 0.05;      // 0.05s stabilization wait
    bool current_gate_state = true;      // Start with Gate ON as requested

    auto start_time = std::chrono::steady_clock::now();
    int cycle_count = 0;
    int frames_in_period = 0;
    bool is_measuring = false;

    // Initial Gate Control (On)
    auto ctrl = std::make_shared<dai::GateControl>(current_gate_state, -1);
    gateControlQueue->send(ctrl);

    std::cout << "--- Starting Test: Gate is ON ---" << std::endl;

    while(pipeline.isRunning() && cycle_count < num_cycles) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;

        if(elapsed.count() >= period_duration) {
            std::cout << "\n--- Cycle " << cycle_count << ": Toggling Gate to " << (current_gate_state ? "ON" : "OFF") << " ---" << std::endl;
            if(current_gate_state) {
                CHECK(frames_in_period > 30);
                std::cout << "[VERIFY] Gate ON period finished. Frames received: " << frames_in_period << std::endl;
            } else {
                CHECK(frames_in_period == 0);
                std::cout << "[VERIFY] Gate OFF period finished. Frames received: " << frames_in_period << std::endl;
            }

            // Toggle State
            current_gate_state = !current_gate_state;
            ctrl->open = current_gate_state;
            gateControlQueue->send(ctrl);

            // Reset Period
            start_time = now;
            frames_in_period = 0;
            is_measuring = false;
            cycle_count++;
            continue;
        }
        auto frame = cameraQueue->tryGet<dai::ImgFrame>();

        if(elapsed.count() >= wait_delay) {
            is_measuring = true;
        }

        if(is_measuring) {
            if(frame) {
                if(is_measuring) {
                    frames_in_period++;
                }
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

    pipeline.start();

    // Test Parameters
    const int num_messages = 8;
    const int num_cycles = 4;            // Number of times to toggle
    const double period_duration = 2.0;  // 2 seconds per state
    const double wait_delay = 0.05;      // 0.05s stabilization wait

    auto start_time = std::chrono::steady_clock::now();
    int cycle_count = 0;
    int frames_in_period = 0;

    // Initial Gate Control (On)
    auto ctrl = std::make_shared<dai::GateControl>(true, num_messages);
    gateControlQueue->send(ctrl);

    std::cout << "--- Starting Test: Gate is ON ---" << std::endl;

    while(pipeline.isRunning() && cycle_count < num_cycles) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;

        if(elapsed.count() >= period_duration) {
            if(cycle_count == 0) {
                CHECK(frames_in_period >= num_messages);
            } else {
                CHECK(frames_in_period == num_messages);
            }
            gateControlQueue->send(ctrl);
            std::cout << "[VERIFY] Gate ON for N messages period finished. Frames received: " << frames_in_period << std::endl;

            // Reset Period
            start_time = now;
            frames_in_period = 0;
            cycle_count++;
            continue;
        }
        auto frame = cameraQueue->tryGet<dai::ImgFrame>();

        if(frame) {
            frames_in_period++;
        }
    }
}
