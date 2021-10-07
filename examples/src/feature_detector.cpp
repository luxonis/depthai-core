#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

void drawFeatures(cv::Mat& img, std::vector<dai::TrackedFeature>& features) {
    static const auto pointColor = cv::Scalar(0, 0, 255);
    static const int circleRadius = 2;
    for(auto& feature : features) {
        cv::circle(img, cv::Point(feature.position.x, feature.position.y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
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
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto xoutPassthroughFrameLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutPassthroughFrameRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto xinTrackedFeaturesConfig = pipeline.create<dai::node::XLinkIn>();

    xoutPassthroughFrameLeft->setStreamName("passthroughFrameLeft");
    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutPassthroughFrameRight->setStreamName("passthroughFrameRight");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xinTrackedFeaturesConfig->setStreamName("trackedFeaturesConfig");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->passthroughInputImage.link(xoutPassthroughFrameLeft->input);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerLeft->inputConfig);

    monoRight->out.link(featureTrackerRight->inputImage);
    featureTrackerRight->passthroughInputImage.link(xoutPassthroughFrameRight->input);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerRight->inputConfig);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    printf("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation! \n");

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, false);
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto passthroughImageRightQueue = device.getOutputQueue("passthroughFrameRight", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);

    auto inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig");

    while(true) {
        auto inPassthroughFrameLeft = passthroughImageLeftQueue->get<dai::ImgFrame>();
        cv::Mat passthroughFrameLeft = inPassthroughFrameLeft->getFrame();
        cv::Mat leftFrame;
        cv::cvtColor(passthroughFrameLeft, leftFrame, cv::COLOR_GRAY2BGR);

        auto inPassthroughFrameRight = passthroughImageRightQueue->get<dai::ImgFrame>();
        cv::Mat passthroughFrameRight = inPassthroughFrameRight->getFrame();
        cv::Mat rightFrame;
        cv::cvtColor(passthroughFrameRight, rightFrame, cv::COLOR_GRAY2BGR);

        auto trackedFeaturesLeft = outputFeaturesLeftQueue->get<dai::TrackedFeatures>()->trackedFeatures;
        drawFeatures(leftFrame, trackedFeaturesLeft);

        auto trackedFeaturesRight = outputFeaturesRightQueue->get<dai::TrackedFeatures>()->trackedFeatures;
        drawFeatures(rightFrame, trackedFeaturesRight);

        // Show the frame
        cv::imshow("left", leftFrame);
        cv::imshow("right", rightFrame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 's') {
            if(featureTrackerConfig.motionEstimator.type == dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW) {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
                printf("Switching to hardware accelerated motion estimation \n");
            } else {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
                printf("Switching to Lucas-Kanade optical flow \n");
            }
            auto cfg = dai::FeatureTrackerConfig();
            cfg.set(featureTrackerConfig);
            inputFeatureTrackerConfigQueue->send(cfg);
        }
    }
    return 0;
}
