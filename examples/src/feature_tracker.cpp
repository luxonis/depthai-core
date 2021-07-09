#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "unordered_map"
#include "unordered_set"

using featureIdType = decltype(dai::Point2f::x);

static std::unordered_set<featureIdType> trackedIDs;
static std::unordered_map<featureIdType, std::deque<dai::Point2f>> trackedFeaturesPath;

const auto lineColor = cv::Scalar(200, 0, 200);
const auto pointColor = cv::Scalar(0, 0, 255);
const int circleRadius = 2;
const int maxTrackedFeaturesPathLength = 30;
// for how many frames the feature is tracked
int trackedFeaturesPathLength = 10;

static void trackFeaturePath(std::vector<dai::TrackedFeatures>& features) {
    std::unordered_set<featureIdType> newTrackedIDs;
    for(auto& currentFeature : features) {
        auto currentID = currentFeature.id;
        newTrackedIDs.insert(currentID);

        if(!trackedFeaturesPath.count(currentID)) {
            trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
        }
        std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

        path.push_back(currentFeature.position);
        while(path.size() > std::max(1, trackedFeaturesPathLength)) {
            path.pop_front();
        }
    }

    std::unordered_set<featureIdType> featuresToRemove;
    for(auto& oldId : trackedIDs) {
        if(!newTrackedIDs.count(oldId)) {
            featuresToRemove.insert(oldId);
        }
    }

    for(auto& id : featuresToRemove) {
        trackedFeaturesPath.erase(id);
    }

    trackedIDs = newTrackedIDs;
}

static void drawFeatures(cv::Mat& img) {
    // For every feature,
    for(auto& featurePath : trackedFeaturesPath) {
        std::deque<dai::Point2f>& path = featurePath.second;
        int j = 0;
        // Draw the feature movement path.
        for(j = 0; j < path.size() - 1; j++) {
            auto src = cv::Point(path[j].x, path[j].y);
            auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
            cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
        }

        cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
    }
}

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();

    auto xoutPassthroughFrameLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();

    xoutPassthroughFrameLeft->setStreamName("passthroughFrameLeft");
    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);

    featureTrackerLeft->passthroughInputImage.link(xoutPassthroughFrameLeft->input);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, false);
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);

    const auto leftWindowName = "left";
    cv::namedWindow(leftWindowName);
    cv::createTrackbar("Feature tracking duration (frames)", leftWindowName, &trackedFeaturesPathLength, maxTrackedFeaturesPathLength, nullptr);

    while(true) {
        auto inPassthroughFrameLeft = passthroughImageLeftQueue->get<dai::ImgFrame>();

        cv::Mat passthroughFrameLeft = inPassthroughFrameLeft->getFrame();
        cv::Mat frame;
        cv::cvtColor(passthroughFrameLeft, frame, cv::COLOR_GRAY2BGR);

        auto trackedFeaturesLeft = outputFeaturesLeftQueue->get<dai::FeatureTrackerData>()->trackedFeatures;
        trackFeaturePath(trackedFeaturesLeft);
        drawFeatures(frame);

        // Show the frame
        cv::imshow(leftWindowName, frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
    return 0;
}
