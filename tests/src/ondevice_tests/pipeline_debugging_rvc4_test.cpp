#include <fmt/base.h>

#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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
        REQUIRE(nodeState.mainLoopTiming.isValid());
        if(!node->getInputs().empty()) REQUIRE(nodeState.inputsGetTiming.isValid());
        if(!node->getOutputs().empty()) REQUIRE(nodeState.outputsSendTiming.isValid());
        for(const auto& [inputName, inputState] : nodeState.inputStates) {
            if(std::string(node->getName()) == "ObjectTracker" && inputName == "inputConfig") continue;  // This example does not use inputConfig
            if(std::string(node->getName()) == "ObjectTracker" && inputName == "inputDetectionFrame")
                continue;  // This example does not use inputDetectionFrame
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
