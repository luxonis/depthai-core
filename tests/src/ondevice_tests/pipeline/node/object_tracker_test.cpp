#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

#define VIDEO_DURATION_SECONDS 10

template <typename T, typename Container, typename KeyFunc>
bool has_duplicates(const Container& items, KeyFunc key_func) {
    std::unordered_set<T> seen;
    for(const auto& item : items) {
        auto key = key_func(item);
        if(!seen.insert(key).second) {
            return true;  // duplicate found
        }
    }
    return false;  // no duplicates
}

TEST_CASE("Object Tracker smallest ID assignment policy") {
    // Create pipeline
    dai::Pipeline pipeline;

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

    int counter = 0;

    auto start = std::chrono::steady_clock::now();
    while(pipeline.isRunning() && std::chrono::steady_clock::now() - start < std::chrono::seconds(VIDEO_DURATION_SECONDS)) {
        auto track = tracklets->get<dai::Tracklets>();
        REQUIRE(track != nullptr);
        REQUIRE(!has_duplicates<int>(track->tracklets, [](const dai::Tracklet& t) { return t.id; }));
        counter += track->tracklets.size();
    }
    REQUIRE(counter > 0);  // Ensure that at least some tracklets were processed
}

TEST_CASE("Object Tracker unique ID assignment policy") {
    // Create pipeline
    dai::Pipeline pipeline;

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
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);

    // Create output queues
    auto tracklets = objectTracker->out.createOutputQueue();

    // Link nodes
    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);

    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);

    // Start pipeline
    pipeline.start();

    int counter = 0;

    auto start = std::chrono::steady_clock::now();
    while(pipeline.isRunning() && std::chrono::steady_clock::now() - start < std::chrono::seconds(VIDEO_DURATION_SECONDS)) {
        auto track = tracklets->get<dai::Tracklets>();
        REQUIRE(track != nullptr);
        REQUIRE(!has_duplicates<int>(track->tracklets, [](const dai::Tracklet& t) { return t.id; }));
        counter += track->tracklets.size();
    }
    REQUIRE(counter > 0);  // Ensure that at least some tracklets were processed
}

TEST_CASE("Object Tracker transformation") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto* camOut = cam->requestOutput({1240, 720});

    // Create spatial detection network
    dai::NNModelDescription modelDescription{"yolov6-nano"};
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(cam, modelDescription);
    detectionNetwork->setConfidenceThreshold(0.6f);
    detectionNetwork->input.setBlocking(false);

    // Create object tracker
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();

    // Link nodes
    camOut->link(objectTracker->inputTrackerFrame);

    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);

    // Create output queues
    auto tracklets = objectTracker->out.createOutputQueue();
    auto trackerFrameQ = objectTracker->passthroughTrackerFrame.createOutputQueue();

    // Start pipeline
    pipeline.start();

    auto start = std::chrono::steady_clock::now();
    while(pipeline.isRunning() && std::chrono::steady_clock::now() - start < std::chrono::seconds(VIDEO_DURATION_SECONDS)) {
        auto track = tracklets->get<dai::Tracklets>();
        auto frame = trackerFrameQ->get<dai::ImgFrame>();
        REQUIRE(track != nullptr);
        REQUIRE(track->transformation.getSize() == frame->transformation.getSize());
        REQUIRE(track->transformation.getSourceSize() == frame->transformation.getSourceSize());
        REQUIRE(track->transformation.getIntrinsicMatrix() == frame->transformation.getIntrinsicMatrix());
    }
}
