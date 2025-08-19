#include <chrono>
#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/pipeline/datatype/Tracklets.hpp"

int main() {
    bool fullFrameTracking = false;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Create stereo node
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto leftOutput = monoLeft->requestOutput(std::make_pair(640, 400));
    auto rightOutput = monoRight->requestOutput(std::make_pair(640, 400));
    leftOutput->link(stereo->left);
    rightOutput->link(stereo->right);

    // Create spatial detection network
    dai::NNModelDescription modelDescription{"yolov6-nano"};
    auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>()->build(camRgb, stereo, modelDescription);
    spatialDetectionNetwork->setConfidenceThreshold(0.6f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // Create object tracker
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    objectTracker->setDetectionLabelsToTrack({0});  // track only person
    objectTracker->setTrackerType(dai::TrackerType::SHORT_TERM_IMAGELESS);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    // Create output queues
    auto preview = objectTracker->passthroughTrackerFrame.createOutputQueue();
    auto tracklets = objectTracker->out.createOutputQueue();

    // Link nodes
    if(fullFrameTracking) {
        camRgb->requestFullResolutionOutput()->link(objectTracker->inputTrackerFrame);
        objectTracker->inputTrackerFrame.setBlocking(false);
        objectTracker->inputTrackerFrame.setMaxSize(1);
    } else {
        spatialDetectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    }

    spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    spatialDetectionNetwork->out.link(objectTracker->inputDetections);

    // Start pipeline
    pipeline.start();

    // FPS calculation variables
    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0;
    cv::Scalar color(255, 255, 255);

    while(pipeline.isRunning()) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto track = tracklets->get<dai::Tracklets>();

        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if(elapsed >= 1) {
            fps = counter / static_cast<float>(elapsed);
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat frame = imgFrame->getCvFrame();
        auto trackletsData = track->tracklets;

        for(const auto& t : trackletsData) {
            auto roi = t.roi.denormalize(frame.cols, frame.rows);
            int x1 = static_cast<int>(roi.topLeft().x);
            int y1 = static_cast<int>(roi.topLeft().y);
            int x2 = static_cast<int>(roi.bottomRight().x);
            int y2 = static_cast<int>(roi.bottomRight().y);

            std::string label;
            try {
                label = spatialDetectionNetwork->getClasses().value()[t.label];
            } catch(...) {
                label = std::to_string(t.label);
            }

            cv::putText(frame, label, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::putText(frame, "ID: " + std::to_string(t.id), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::putText(frame,
                        std::string(t.status == dai::Tracklet::TrackingStatus::LOST ? "LOST" : "TRACKED"),
                        cv::Point(x1 + 10, y1 + 50),
                        cv::FONT_HERSHEY_TRIPLEX,
                        0.5,
                        color);
            cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, cv::FONT_HERSHEY_SIMPLEX);

            cv::putText(frame,
                        "X: " + std::to_string(static_cast<int>(t.spatialCoordinates.x)) + " mm",
                        cv::Point(x1 + 10, y1 + 65),
                        cv::FONT_HERSHEY_TRIPLEX,
                        0.5,
                        color);
            cv::putText(frame,
                        "Y: " + std::to_string(static_cast<int>(t.spatialCoordinates.y)) + " mm",
                        cv::Point(x1 + 10, y1 + 80),
                        cv::FONT_HERSHEY_TRIPLEX,
                        0.5,
                        color);
            cv::putText(frame,
                        "Z: " + std::to_string(static_cast<int>(t.spatialCoordinates.z)) + " mm",
                        cv::Point(x1 + 10, y1 + 95),
                        cv::FONT_HERSHEY_TRIPLEX,
                        0.5,
                        color);
        }

        cv::putText(frame, "NN fps: " + std::to_string(fps).substr(0, 4), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("tracker", frame);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
