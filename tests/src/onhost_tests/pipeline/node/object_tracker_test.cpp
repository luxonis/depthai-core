#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/utility/Serialization.hpp"

TEST_CASE("ObjectTracker preserves source detections", "[object-tracker]") {
    const bool spatial = GENERATE(false, true);
    const bool trackingPerClass = GENERATE(false, true);
    const bool legacyCoordinates = GENERATE(false, true);
    CAPTURE(spatial, trackingPerClass, legacyCoordinates);

    dai::Pipeline pipeline(false);
    auto tracker = pipeline.create<dai::node::ObjectTracker>();
    tracker->setRunOnHost(true);
    tracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
    tracker->setTrackingPerClass(trackingPerClass);
    tracker->setTrackletBirthThreshold(1);
    tracker->setTrackerThreshold(0.5f);

    auto trackerInput = tracker->inputTrackerFrame.createInputQueue();
    auto detectionFrameInput = tracker->inputDetectionFrame.createInputQueue();
    auto detectionsInput = tracker->inputDetections.createInputQueue();
    auto output = tracker->out.createOutputQueue();
    pipeline.start();

    std::array<int32_t, 2> ids{-1, -1};
    std::vector<dai::ImgDetection> expected;
    for(int frameIndex = 0; frameIndex < 4; ++frameIndex) {
        CAPTURE(frameIndex);
        auto trackerFrame = std::make_shared<dai::ImgFrame>();
        trackerFrame->setWidth(320);
        trackerFrame->setHeight(240);
        trackerFrame->setType(dai::ImgFrame::Type::BGR888p);
        trackerFrame->setData(std::vector<uint8_t>(320 * 240 * 3));
        trackerFrame->setSequenceNum(frameIndex);
        trackerFrame->setTimestamp(std::chrono::steady_clock::now());
        trackerFrame->transformation = dai::ImgTransformation(320, 240);

        auto detectionFrame = std::make_shared<dai::ImgFrame>();
        detectionFrame->setWidth(160);
        detectionFrame->setHeight(120);
        detectionFrame->setType(dai::ImgFrame::Type::BGR888p);
        detectionFrame->setData(std::vector<uint8_t>(160 * 120 * 3));
        detectionFrame->transformation = trackerFrame->transformation;
        detectionFrame->transformation.addScale(0.5f, 0.5f);

        auto detections = std::make_shared<dai::ImgDetections>();
        auto spatialDetections = std::make_shared<dai::SpatialImgDetections>();
        if(frameIndex < 3) {
            expected.clear();
            // A filtered detection verifies that source association survives filtering.
            detections->detections.emplace_back(dai::RotatedRect(dai::Rect(0.5f, 0.5f, 0.1f, 0.1f, true)), 0.1f, 99);
            spatialDetections->detections.emplace_back(detections->detections.front().getBoundingBox(), dai::Point3f(0, 0, 2000), 0.1f, 99);
            for(int i = 0; i < 2; ++i) {
                auto box = dai::RotatedRect(dai::Rect(-8.0f + 64.0f * i + 2.0f * frameIndex, 12, 32, 24, false));
                // Mix representations in each message, then switch them on the same tracks.
                if((i + frameIndex) % 2 == 0) box = box.normalize(160, 120);
                dai::ImgDetection detection(box, "object", 0.9f, i + 1);
                detection.setKeypoints(std::vector<dai::Point2f>{box.center});
                if(legacyCoordinates) detection.boundingBox.reset();
                detections->detections.push_back(detection);
                dai::SpatialImgDetection spatialDetection(box, dai::Point3f(10, 20, 2000), "object", 0.9f, i + 1);
                if(legacyCoordinates) spatialDetection.boundingBox.reset();
                spatialDetections->detections.push_back(spatialDetection);
                expected.push_back(spatial ? spatialDetection.getImgDetection() : detection);
            }
        }
        detectionFrameInput->send(detectionFrame);
        if(spatial) {
            detectionsInput->send(spatialDetections);
        } else {
            detectionsInput->send(detections);
        }
        trackerInput->send(trackerFrame);

        bool timedOut = false;
        auto result = output->get<dai::Tracklets>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(result != nullptr);
        REQUIRE(result->getSequenceNum() == frameIndex);
        REQUIRE(result->tracklets.size() == expected.size());
        for(const auto& tracklet : result->tracklets) {
            REQUIRE(tracklet.label >= 1);
            REQUIRE(tracklet.label <= 2);
            const auto index = tracklet.label - 1;
            REQUIRE(dai::utility::serialize(tracklet.srcImgDetection) == dai::utility::serialize(expected[index]));
            if(frameIndex == 0) ids[index] = tracklet.id;
            REQUIRE(tracklet.id == ids[index]);
            REQUIRE(tracklet.roi.isNormalized());
            // Tracking still uses remapped, clipped pixels, independently of the source representation.
            const float x = -16.0f + 128.0f * index + 4.0f * std::min(frameIndex, 2);
            REQUIRE(tracklet.roi.x == Catch::Approx(std::max(0.0f, x) / 320.0f));
            REQUIRE(tracklet.roi.y == Catch::Approx(0.1f));
            REQUIRE(tracklet.roi.width == Catch::Approx((x + 64.0f - std::max(0.0f, x)) / 320.0f));
            REQUIRE(tracklet.roi.height == Catch::Approx(0.2f));
            if(frameIndex == 3) REQUIRE(tracklet.status == dai::Tracklet::TrackingStatus::LOST);
            REQUIRE(tracklet.spatialCoordinates.z == (spatial ? 2000.0f : 0.0f));
        }
    }
    pipeline.stop();
}
