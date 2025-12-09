#include <fmt/base.h>

#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

#define VIDEO_DURATION_SECONDS 5

TEST_CASE("Object Tracker Pipeline Debugging") {
    std::vector<int64_t> skipNodeIds;
    if(!dai::utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false)) {
        skipNodeIds = {
            9,   // XLinkOutHost of merge outRequest
            10,  // XLinkIn of aggregation request
            11,  // XLinkOut of aggregation out
            12,  // XLinkInHost of merge inputDevice
        };
    } else {
        skipNodeIds = {
            11,
            12,
            15,
            16,
        };
    }

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.enablePipelineDebugging();

    // Define sources and outputs
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(VIDEO_PATH);
    replay->setLoop(false);

    // Create spatial detection network
    dai::NNModelDescription modelDescription{"yolov6-nano"};
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(replay, modelDescription);
    detectionNetwork->setConfidenceThreshold(0.6f);
    detectionNetwork->input.setBlocking(false);

    // Create object tracker
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    objectTracker->setDetectionLabelsToTrack({0});  // track only person
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    // Create output queues
    auto tracklets = objectTracker->out.createOutputQueue();

    // Link nodes
    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);

    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);

    // Start pipeline
    pipeline.start();

    auto start = std::chrono::steady_clock::now();
    while(pipeline.isRunning() && std::chrono::steady_clock::now() - start < std::chrono::seconds(VIDEO_DURATION_SECONDS)) {
        auto track = tracklets->get<dai::Tracklets>();
    }

    auto state = pipeline.getPipelineState().nodes().detailed();

    for(const auto& [nodeId, nodeState] : state.nodeStates) {
        if(std::find(skipNodeIds.begin(), skipNodeIds.end(), nodeId) != skipNodeIds.end()) continue;

        auto node = pipeline.getNode(nodeId);
        // Not implemented yet on RVC2 REQUIRE(nodeState.mainLoopTiming.isValid());
        // Not implemented yet on RVC2 if(!node->getInputs().empty()) REQUIRE(nodeState.inputsGetTiming.isValid());
        // Not implemented yet on RVC2 if(!node->getOutputs().empty()) REQUIRE(nodeState.outputsSendTiming.isValid());
        for(const auto& [inputName, inputState] : nodeState.inputStates) {
            if(std::string(node->getName()) == "ObjectTracker" && inputName == "inputConfig") continue;  // This example does not use inputConfig
            REQUIRE(inputState.timing.isValid());
        }
        for(const auto& [outputName, outputState] : nodeState.outputStates) {
            REQUIRE(outputState.timing.isValid());
        }
        for(const auto& [otherName, otherTiming] : nodeState.otherTimings) {
            REQUIRE(otherTiming.isValid());
        }
    }
}

TEST_CASE("FPS check") {
    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.enablePipelineDebugging();

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->build(dai::CameraBoardSocket::CAM_A);

    auto monoLeft = pipeline.create<dai::node::Camera>();
    monoLeft->build(dai::CameraBoardSocket::CAM_B);

    auto monoRight = pipeline.create<dai::node::Camera>();
    monoRight->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>();

    // Configure stereo node
    stereo->setExtendedDisparity(true);
    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform == dai::Platform::RVC2) {
        stereo->setOutputSize(640, 400);
    }

    // Configure spatial detection network
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    auto passthroughDepthQueue = spatialDetectionNetwork->passthroughDepth.createOutputQueue();
    auto out = spatialDetectionNetwork->out.createOutputQueue();

    // Set up model
    dai::NNModelDescription modelDesc;
    modelDesc.model = "yolov6-nano";
    spatialDetectionNetwork->build(camRgb, stereo, modelDesc, 8);  // 30 FPS

    // Linking
    monoLeft->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 8)->link(stereo->left);
    monoRight->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 8)->link(stereo->right);

    // Start pipeline
    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(VIDEO_DURATION_SECONDS));

    auto state = pipeline.getPipelineState().nodes().detailed();

    int gotNodes = 0;
    for(const auto& [nodeId, nodeState] : state.nodeStates) {
        auto node = pipeline.getNode(nodeId);
        if(std::string(node->getName()) == "Camera") {
            ++gotNodes;
            // Not implemented on RVC2 REQUIRE(nodeState.mainLoopTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.outputsSendTiming.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("0").isValid());
            REQUIRE(nodeState.outputStates.at("0").timing.fps == Catch::Approx(8.0).margin(2.0));
        }
        if(std::string(node->getName()) == "NeuralNetwork") {
            ++gotNodes;
            // Not implemented on RVC2 REQUIRE(nodeState.mainLoopTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.inputsGetTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.outputsSendTiming.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("in").isValid());
            REQUIRE(nodeState.inputStates.at("in").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("passthrough").isValid());
            REQUIRE(nodeState.outputStates.at("passthrough").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("out").isValid());
            REQUIRE(nodeState.outputStates.at("out").timing.fps == Catch::Approx(8.0).margin(2.0));
        }
        if(std::string(node->getName()) == "StereoDepth") {
            ++gotNodes;
            // Not implemented on RVC2 REQUIRE(nodeState.mainLoopTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.inputsGetTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.outputsSendTiming.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("left").isValid());
            REQUIRE(nodeState.inputStates.at("left").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("right").isValid());
            REQUIRE(nodeState.inputStates.at("right").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("depth").isValid());
            REQUIRE(nodeState.outputStates.at("depth").timing.fps == Catch::Approx(8.0).margin(2.0));
        }
        if(std::string(node->getName()) == "DetectionParser") {
            ++gotNodes;
            // Not implemented on RVC2 REQUIRE(nodeState.mainLoopTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.inputsGetTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.outputsSendTiming.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("in").isValid());
            REQUIRE(nodeState.inputStates.at("in").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("out").isValid());
            REQUIRE(nodeState.outputStates.at("out").timing.fps == Catch::Approx(8.0).margin(2.0));
        }
        if(std::string(node->getName()) == "SpatialDetectionNetwork") {
            ++gotNodes;
            // Not implemented on RVC2 REQUIRE(nodeState.mainLoopTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.inputsGetTiming.fps == Catch::Approx(8.0).margin(2.0));
            // Not implemented on RVC2 REQUIRE(nodeState.outputsSendTiming.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("inputDepth").isValid());
            REQUIRE(nodeState.inputStates.at("inputDepth").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("inputDetections").isValid());
            REQUIRE(nodeState.inputStates.at("inputDetections").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.inputStates.at("inputImg").isValid());
            REQUIRE(nodeState.inputStates.at("inputImg").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("passthroughDepth").isValid());
            REQUIRE(nodeState.outputStates.at("passthroughDepth").timing.fps == Catch::Approx(8.0).margin(2.0));
            REQUIRE(nodeState.outputStates.at("out").isValid());
            REQUIRE(nodeState.outputStates.at("out").timing.fps == Catch::Approx(8.0).margin(2.0));
        }
    }

    REQUIRE(gotNodes == 7);  // 3 cameras, stereo, neural network, detection parser, spatial detection network

    pipeline.stop();
}
