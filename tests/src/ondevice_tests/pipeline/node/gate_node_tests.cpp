#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "buffer_test_utils.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Gate.hpp"

TEST_CASE("ImgFrame metadata survives a device round trip") {
    constexpr unsigned int FRAME_WIDTH = 128;
    constexpr unsigned int FRAME_HEIGHT = 32;
    constexpr unsigned int SOURCE_WIDTH = FRAME_WIDTH * 2;
    constexpr unsigned int SOURCE_HEIGHT = FRAME_HEIGHT * 2;

    dai::Pipeline pipeline;
    auto gate = pipeline.create<dai::node::Gate>();
    gate->initialConfig->open = true;
    auto input = gate->input.createInputQueue();
    auto output = gate->output.createOutputQueue();

    auto sent = std::make_shared<dai::ImgFrame>();
    sent->setSourceSize(SOURCE_WIDTH, SOURCE_HEIGHT);
    sent->setSize(FRAME_WIDTH, FRAME_HEIGHT);
    sent->setType(dai::ImgFrame::Type::GRAY8);
    sent->setStride(FRAME_WIDTH);
    sent->sourceFb.bytesPP = 1;
    auto transformation = dai::ImgTransformation(SOURCE_WIDTH, SOURCE_HEIGHT);
    transformation.setDistortionCoefficients(std::vector<float>(14, 0.0f));
    transformation.addScale(0.5f, 0.5f);
    sent->setTransformation(transformation);
    sent->setData(std::vector<uint8_t>(FRAME_WIDTH * FRAME_HEIGHT, 0x5a));
    sent->setSequenceNum(42);
    sent->setInstanceNum(7);
    sent->setCategory(3);
    sent->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(123456789)));
    sent->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(234567890)));
    sent->setTimestampSystem(
        std::chrono::system_clock::time_point(std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::nanoseconds(345678901))));
    sent->cam.exposureTimeUs = 1234;
    sent->cam.sensitivityIso = 321;
    sent->cam.lensPosition = 12;
    sent->cam.lensPositionRaw = 1.25f;
    sent->cam.wbColorTemp = 4500;
    sent->cam.fsync = dai::ImgFrame::Fsync::INPUT;
    sent->cam.sensorMode = 2;
    sent->cam.fps = 30.0f;
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
    REQUIRE(received->fb.p1Offset == sent->fb.p1Offset);
    REQUIRE(received->fb.p2Offset == sent->fb.p2Offset);
    REQUIRE(received->fb.p3Offset == sent->fb.p3Offset);
    REQUIRE(received->getSourceWidth() == sent->getSourceWidth());
    REQUIRE(received->getSourceHeight() == sent->getSourceHeight());
    REQUIRE(received->sourceFb.type == sent->sourceFb.type);
    REQUIRE(received->sourceFb.stride == sent->sourceFb.stride);
    REQUIRE(received->sourceFb.bytesPP == sent->sourceFb.bytesPP);
    REQUIRE(received->sourceFb.p1Offset == sent->sourceFb.p1Offset);
    REQUIRE(received->sourceFb.p2Offset == sent->sourceFb.p2Offset);
    REQUIRE(received->sourceFb.p3Offset == sent->sourceFb.p3Offset);
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
    REQUIRE(received->event == sent->event);
    REQUIRE(received->getTimestamp() == sent->getTimestamp());
    REQUIRE(received->getTimestampDevice() == sent->getTimestampDevice());
    REQUIRE(received->getTimestampSystem() == sent->getTimestampSystem());
    REQUIRE(received->cam.exposureTimeUs == sent->cam.exposureTimeUs);
    REQUIRE(received->cam.sensitivityIso == sent->cam.sensitivityIso);
    REQUIRE(received->cam.lensPosition == sent->cam.lensPosition);
    REQUIRE(received->cam.lensPositionRaw == sent->cam.lensPositionRaw);
    REQUIRE(received->cam.wbColorTemp == sent->cam.wbColorTemp);
    REQUIRE(received->cam.fsync == sent->cam.fsync);
    REQUIRE(received->cam.sensorMode == sent->cam.sensorMode);
    REQUIRE(received->cam.fps == sent->cam.fps);
    REQUIRE(received->cam.sensorTemperatureC == sent->cam.sensorTemperatureC);
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

TEST_CASE("Device Gate preserves buffers and mixed groups", "[.][transport]") {
    using namespace buffer_test;
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    REQUIRE(device->getDeviceId() == device->getDeviceInfo().getDeviceId());
    auto gate = pipeline.create<dai::node::Gate>();
    gate->setRunOnHost(false);
    gate->initialConfig->open = true;
    pipeline.getAssetManager().set("pipeline-test", std::vector<uint8_t>(1024, 0xa5));
    gate->getAssetManager().set("node-test", std::vector<uint8_t>{0x12, 0x34, 0x56}, 64);
    auto input = gate->input.createInputQueue();
    auto output = gate->output.createOutputQueue();
    pipeline.start();

    input->send(makeBuffer(31));
    requireBuffer(take<dai::Buffer>(output), 31);

    auto large = makeBuffer(44);
    large->setData(std::vector<uint8_t>(1024 * 1024, 0xa5));
    input->send(large);
    auto receivedLarge = take<dai::Buffer>(output);
    REQUIRE(receivedLarge->getData().size() == 1024 * 1024);
    REQUIRE(std::count(receivedLarge->getData().begin(), receivedLarge->getData().end(), 0xa5) == 1024 * 1024);
    REQUIRE(receivedLarge->getSequenceNum() == 44);

    auto group = std::make_shared<dai::MessageGroup>();
    group->setSequenceNum(99);
    group->add("bytes", makeBuffer(43));
    group->add("image", makeImage());
    input->send(group);
    auto receivedGroup = take<dai::MessageGroup>(output);
    REQUIRE(receivedGroup->getNumMessages() == 2);
    REQUIRE(receivedGroup->getSequenceNum() == 99);
    REQUIRE(receivedGroup->getMessageNames() == std::vector<std::string>{"bytes", "image"});
    requireBuffer(receivedGroup->get<dai::Buffer>("bytes"), 43);
    requireImage(receivedGroup->get<dai::ImgFrame>("image"));

    pipeline.stop();
    pipeline.wait();
    REQUIRE_FALSE(pipeline.isRunning());
    REQUIRE(device->isClosed());
}

TEST_CASE("Device Gate retains input across finite and indefinite grants", "[.][grants]") {
    using namespace buffer_test;
    dai::Pipeline pipeline;
    auto gate = pipeline.create<dai::node::Gate>();
    gate->setRunOnHost(false);
    gate->initialConfig->open = false;
    auto input = gate->input.createInputQueue();
    auto control = gate->inputControl.createInputQueue();
    auto output = gate->output.createOutputQueue();
    pipeline.start();

    input->send(makeBuffer(10));
    requireNoMessage(output);
    control->send(dai::GateControl::openGate(2));
    requireBuffer(take<dai::Buffer>(output), 10);
    input->send(makeBuffer(11));
    requireBuffer(take<dai::Buffer>(output), 11);
    input->send(makeBuffer(12));
    requireNoMessage(output);

    control->send(dai::GateControl::openGate());
    requireBuffer(take<dai::Buffer>(output), 12);

    control->send(dai::GateControl::closeGate());
    // Control and data use separate streams, so let the close land before the next input
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    input->send(makeBuffer(13));
    requireNoMessage(output);
    control->send(dai::GateControl::openGate(1));
    requireBuffer(take<dai::Buffer>(output), 13);
}
