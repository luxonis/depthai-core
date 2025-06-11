#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <set>

#include "depthai/depthai.hpp"

class FeatureTrackerDrawer {
   public:
    static const cv::Scalar lineColor;
    static const cv::Scalar pointColor;
    static const int circleRadius = 2;
    static const int maxTrackedFeaturesPathLength = 30;
    static int trackedFeaturesPathLength;

    FeatureTrackerDrawer(const std::string& trackbarName, const std::string& windowName) : trackbarName(trackbarName), windowName(windowName) {
        cv::namedWindow(windowName);
        cv::createTrackbar(trackbarName, windowName, &trackedFeaturesPathLength, maxTrackedFeaturesPathLength, onTrackBar, this);
    }

    static void onTrackBar(int val, void* userdata) {
        trackedFeaturesPathLength = val;
    }

    void trackFeaturePath(const std::vector<dai::TrackedFeature>& features) {
        std::set<int> newTrackedIDs;

        for(const auto& currentFeature : features) {
            int currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(trackedFeaturesPath.find(currentID) == trackedFeaturesPath.end()) {
                trackedFeaturesPath[currentID] = std::deque<dai::Point2f>();
            }

            auto& path = trackedFeaturesPath[currentID];
            path.push_back(currentFeature.position);

            while(path.size() > std::max(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        // Remove features that are no longer tracked
        std::set<int> featuresToRemove;
        for(const auto& oldId : trackedIDs) {
            if(newTrackedIDs.find(oldId) == newTrackedIDs.end()) {
                featuresToRemove.insert(oldId);
            }
        }

        for(const auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
    }

    void drawFeatures(cv::Mat& img) {
        cv::setTrackbarPos(trackbarName.c_str(), windowName.c_str(), trackedFeaturesPathLength);

        for(const auto& [id, path] : trackedFeaturesPath) {
            for(size_t j = 0; j < path.size() - 1; j++) {
                cv::Point src(static_cast<int>(path[j].x), static_cast<int>(path[j].y));
                cv::Point dst(static_cast<int>(path[j + 1].x), static_cast<int>(path[j + 1].y));
                cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
            }

            if(!path.empty()) {
                size_t j = path.size() - 1;
                cv::Point point(static_cast<int>(path[j].x), static_cast<int>(path[j].y));
                cv::circle(img, point, circleRadius, pointColor, -1, cv::LINE_AA, 0);
            }
        }
    }

   private:
    std::string trackbarName;
    std::string windowName;
    std::set<int> trackedIDs;
    std::map<int, std::deque<dai::Point2f>> trackedFeaturesPath;
};

// Initialize static members
const cv::Scalar FeatureTrackerDrawer::lineColor(200, 0, 200);
const cv::Scalar FeatureTrackerDrawer::pointColor(0, 0, 255);
int FeatureTrackerDrawer::trackedFeaturesPathLength = 10;

void onTrackbar(int val, void* userdata) {
    auto* inputConfigQueue = static_cast<std::shared_ptr<dai::InputQueue>*>(userdata);
    try {
        auto cfg = std::make_shared<dai::FeatureTrackerConfig>();
        auto cornerDetector = dai::FeatureTrackerConfig::CornerDetector();
        cornerDetector.numMaxFeatures = cv::getTrackbarPos("numMaxFeatures", "Features");
        cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures;

        auto thresholds = dai::FeatureTrackerConfig::CornerDetector::Thresholds();
        thresholds.initialValue = cv::getTrackbarPos("harrisScore", "Features");
        cornerDetector.thresholds = thresholds;

        cfg->setCornerDetector(cornerDetector);
        if(*inputConfigQueue) {
            (*inputConfigQueue)->send(cfg);
        }
    } catch(const cv::Exception& e) {
        // Ignore OpenCV errors
    }
}

int main() {
    std::cout << "Press 'm' to enable/disable motion estimation!" << std::endl;

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto camOutput = camera->requestOutput(std::make_pair(640, 640), dai::ImgFrame::Type::NV12);

    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::GRAY8);
    camOutput->link(manip->inputImage);

    auto featureTracker = pipeline.create<dai::node::FeatureTracker>();
    featureTracker->initialConfig->setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS);
    featureTracker->initialConfig->setMotionEstimator(false);
    featureTracker->initialConfig->setNumTargetFeatures(256);

    auto motionEstimator = dai::FeatureTrackerConfig::MotionEstimator();
    motionEstimator.enable = true;
    featureTracker->initialConfig->setMotionEstimator(motionEstimator);

    auto cornerDetector = dai::FeatureTrackerConfig::CornerDetector();
    cornerDetector.numMaxFeatures = 256;
    cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures;

    // RVC2 specific setting to allow for more features
    featureTracker->setHardwareResources(2, 2);

    auto outputFeaturePassthroughQueue = camOutput->createOutputQueue();
    auto outputFeatureQueue = featureTracker->outputFeatures.createOutputQueue();
    auto inputConfigQueue = featureTracker->inputConfig.createInputQueue();

    manip->out.link(featureTracker->inputImage);

    auto thresholds = dai::FeatureTrackerConfig::CornerDetector::Thresholds();
    thresholds.initialValue = 20000;  // Default value

    cornerDetector.thresholds = thresholds;
    featureTracker->initialConfig->setCornerDetector(cornerDetector);

    // Create window and trackbars
    cv::namedWindow("Features", cv::WINDOW_NORMAL);
    cv::resizeWindow("Features", 1080, 800);

    cv::createTrackbar("harrisScore", "Features", nullptr, 25000, onTrackbar, &inputConfigQueue);
    cv::createTrackbar("numMaxFeatures", "Features", nullptr, 1024, onTrackbar, &inputConfigQueue);
    cv::setTrackbarPos("harrisScore", "Features", 20000);
    cv::setTrackbarPos("numMaxFeatures", "Features", 256);

    std::string leftWindowName = "Features";
    FeatureTrackerDrawer leftFeatureDrawer("Feature tracking duration (frames)", leftWindowName);

    // Start pipeline
    pipeline.start();

    while(true) {
        auto outputPassthroughImage = outputFeaturePassthroughQueue->get<dai::ImgFrame>();
        if(outputPassthroughImage == nullptr) continue;

        cv::Mat passthroughImage = outputPassthroughImage->getCvFrame();
        auto trackedFeaturesLeft = outputFeatureQueue->get<dai::TrackedFeatures>()->trackedFeatures;

        leftFeatureDrawer.trackFeaturePath(trackedFeaturesLeft);
        leftFeatureDrawer.drawFeatures(passthroughImage);

        cv::imshow(leftWindowName, passthroughImage);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 'm') {
            auto cfg = std::make_shared<dai::FeatureTrackerConfig>();
            auto cornerDetector = dai::FeatureTrackerConfig::CornerDetector();
            cornerDetector.numMaxFeatures = cv::getTrackbarPos("numMaxFeatures", "Features");
            cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures;

            auto thresholds = dai::FeatureTrackerConfig::CornerDetector::Thresholds();
            thresholds.initialValue = cv::getTrackbarPos("harrisScore", "Features");
            cornerDetector.thresholds = thresholds;

            cfg->setCornerDetector(cornerDetector);
            cfg->setMotionEstimator(motionEstimator);

            if(!motionEstimator.enable) {
                motionEstimator.enable = true;
                cfg->setMotionEstimator(motionEstimator);
                std::cout << "Enabling motionEstimator" << std::endl;
            } else {
                motionEstimator.enable = false;
                cfg->setMotionEstimator(motionEstimator);
                std::cout << "Disabling motionEstimator" << std::endl;
            }

            inputConfigQueue->send(cfg);
        }
    }

    return 0;
}