#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Gate.hpp"

TEST_CASE("ImgFrame metadata survives a device round trip") {
    dai::Pipeline pipeline;
    auto gate = pipeline.create<dai::node::Gate>();
    gate->initialConfig->open = true;
    auto input = gate->input.createInputQueue();
    auto output = gate->output.createOutputQueue();

    auto sent = std::make_shared<dai::ImgFrame>();
    sent->setSourceSize(64, 48);
    sent->setSize(32, 24);
    sent->setType(dai::ImgFrame::Type::GRAY8);
    sent->setStride(32);
    sent->sourceFb.bytesPP = 1;
    auto transformation = dai::ImgTransformation(64, 48);
    transformation.setDistortionCoefficients(std::vector<float>(14, 0.0f));
    transformation.addScale(0.5f, 0.5f);
    sent->setTransformation(transformation);
    sent->setData(std::vector<uint8_t>(32 * 24, 0x5a));
    sent->setSequenceNum(42);
    sent->setInstanceNum(7);
    sent->setCategory(3);
    sent->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(123456789)));
    sent->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(234567890)));
    sent->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::nanoseconds(345678901)));
    sent->cam.exposureTimeUs = 1234;
    sent->cam.sensitivityIso = 321;
    REQUIRE(sent->validateTransformations());

    pipeline.start();
    input->send(sent);
    bool timedOut = false;
    auto received = output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(received != nullptr);
    REQUIRE(received->getData().size() == sent->getData().size());
    REQUIRE(std::equal(received->getData().begin(), received->getData().end(), sent->getData().begin()));
    REQUIRE(received->getType() == sent->getType());
    REQUIRE(received->getWidth() == sent->getWidth());
    REQUIRE(received->getHeight() == sent->getHeight());
    REQUIRE(received->getStride() == sent->getStride());
    REQUIRE(received->getBytesPerPixel() == sent->getBytesPerPixel());
    REQUIRE(received->getSourceWidth() == sent->getSourceWidth());
    REQUIRE(received->getSourceHeight() == sent->getSourceHeight());
    REQUIRE(received->sourceFb.stride == sent->sourceFb.stride);
    REQUIRE(received->sourceFb.bytesPP == sent->sourceFb.bytesPP);
    REQUIRE(received->transformation.getSize() == sent->transformation.getSize());
    REQUIRE(received->transformation.getSourceSize() == sent->transformation.getSourceSize());
    REQUIRE(received->transformation.getMatrix() == sent->transformation.getMatrix());
    REQUIRE(received->transformation.getSourceIntrinsicMatrix() == sent->transformation.getSourceIntrinsicMatrix());
    REQUIRE(received->transformation.getDistortionModel() == sent->transformation.getDistortionModel());
    REQUIRE(received->transformation.getDistortionCoefficients() == sent->transformation.getDistortionCoefficients());
    REQUIRE(received->transformation.getExtrinsics().isEqualExtrinsics(sent->transformation.getExtrinsics()));
    REQUIRE(received->validateTransformations());
    REQUIRE(received->getSequenceNum() == sent->getSequenceNum());
    REQUIRE(received->getInstanceNum() == sent->getInstanceNum());
    REQUIRE(received->category == sent->category);
    REQUIRE(received->getTimestamp() == sent->getTimestamp());
    REQUIRE(received->getTimestampDevice() == sent->getTimestampDevice());
    REQUIRE(received->getTimestampSystem() == sent->getTimestampSystem());
    REQUIRE(received->cam.exposureTimeUs == sent->cam.exposureTimeUs);
    REQUIRE(received->cam.sensitivityIso == sent->cam.sensitivityIso);
    pipeline.stop();
}

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
    auto ctrl = std::make_shared<dai::GateControl>(currentGateState, -1, -1);
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
    auto ctrl = std::make_shared<dai::GateControl>(true, numMessages, -1);
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

TEST_CASE("Two Queue from one camera") {
    dai::Pipeline pipeline;

    auto camera = pipeline.create<dai::node::Camera>()->build();

    auto cameraOutGate = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);
    auto cameraOut = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);

    auto gate = pipeline.create<dai::node::Gate>();

    cameraOutGate->link(gate->input);

    auto cameraGateQueue = gate->output.createOutputQueue(8, false);
    auto cameraQueue = cameraOut->createOutputQueue();

    auto gateControlQueue = gate->inputControl.createInputQueue();

    gate->initialConfig->open = false;
    gate->initialConfig->numMessages = -1;

    pipeline.start();

    int msgsFromGateCount = 0;
    int msgsFromCameraCount = 0;

    const double testDuration = 3.0;  // Run for 3 seconds
    auto startTime = std::chrono::steady_clock::now();

    while(pipeline.isRunning()) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - startTime;

        if(elapsed.count() >= testDuration) {
            break;
        }

        auto msgFromGate = cameraGateQueue->tryGet<dai::ImgFrame>();
        auto msgFromCamera = cameraQueue->tryGet<dai::ImgFrame>();

        if(msgFromGate) {
            msgsFromGateCount += 1;
        }
        if(msgFromCamera) {
            msgsFromCameraCount += 1;
        }
    }

    CHECK(msgsFromGateCount == 0);
    CHECK(msgsFromCameraCount > 60);

    std::cout << "Gate frames: " << msgsFromGateCount << " | Camera frames: " << msgsFromCameraCount << std::endl;
}

TEST_CASE("FPS regulation") {
    dai::Pipeline pipeline;

    auto camera = pipeline.create<dai::node::Camera>()->build();

    auto cameraOutGate = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);
    auto cameraOut = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);

    auto gate = pipeline.create<dai::node::Gate>();

    cameraOutGate->link(gate->input);

    auto cameraGateQueue = gate->output.createOutputQueue(8, false);
    auto cameraQueue = cameraOut->createOutputQueue();

    auto gateControlQueue = gate->inputControl.createInputQueue();

    gate->initialConfig->open = true;
    gate->initialConfig->numMessages = -1;
    gate->initialConfig->fps = 15;

    pipeline.start();

    int msgsFromGateCount = 0;
    int msgsFromCameraCount = 0;

    const double testDuration = 3.0;  // Run for 3 seconds
    auto startTime = std::chrono::steady_clock::now();

    while(pipeline.isRunning()) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - startTime;

        if(elapsed.count() >= testDuration) {
            break;
        }

        auto msgFromGate = cameraGateQueue->tryGet<dai::ImgFrame>();
        auto msgFromCamera = cameraQueue->tryGet<dai::ImgFrame>();

        if(msgFromGate) {
            msgsFromGateCount += 1;
        }
        if(msgFromCamera) {
            msgsFromCameraCount += 1;
        }
    }

    CHECK(msgsFromGateCount > 30);
    CHECK(msgsFromGateCount < 50);
    CHECK(msgsFromCameraCount > 60);

    std::cout << "Gate frames: " << msgsFromGateCount << " | Camera frames: " << msgsFromCameraCount << std::endl;
}
