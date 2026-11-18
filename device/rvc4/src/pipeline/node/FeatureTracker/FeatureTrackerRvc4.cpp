#include <bitset>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "FeatureTracker.hpp"
#include "FeatureTrackerAccel.hpp"
#include "FeatureTrackerRvc4Helper.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "pipeline/NodeTiming.hpp"

namespace dai {
namespace gate {

void FeatureTracker::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    CornerDetector cornerDetector;
    DescriptorCalculator descriptorCalculator;
    OpticalFlow opticalFlow;

    using namespace std::chrono;

    std::shared_ptr<ImgFrame> inImagePreviousFrame = nullptr;
    std::shared_ptr<ImgFrame> inImageCurrentFrame = nullptr;

    uint32_t frameCounter = 1;

    std::shared_ptr<FeatureTrackerConfig> inConfig = nullptr;
    FeatureTrackerConfig latestConfig;

    if(featureTrackerConfig) {
        latestConfig = *featureTrackerConfig;

        printConfig(latestConfig, logger);

        if(latestConfig.cornerDetector.thresholds.initialValue) cornerDetector.setThreshold(latestConfig.cornerDetector.thresholds.initialValue);
        cornerDetector.setRobustness(90);  // TODO(Morato) expose
        cornerDetector.setGradientFilterShift(1);

    } else {
        logger->trace("No initial configuration is set");
    }
    // got this value from examples/python/FeatureTracker/feature_tracker.py which had 256-1024 as a reasonable range
    constexpr std::int32_t defaultNumMaxFeatures = 512;
    if(latestConfig.cornerDetector.numMaxFeatures <= 0) latestConfig.cornerDetector.numMaxFeatures = defaultNumMaxFeatures;

    bool initedFpx = false;
    bool initedOf = false;
    bool initedDcm = false;

    bool runAsync = false;

    const uint32_t detectInterval = 1;

    while(mainLoop()) {
        std::shared_ptr<dai::TrackedFeatures> outFeatures = std::make_shared<dai::TrackedFeatures>();

        // Previous frame
        if(frameCounter > 1) {
            inImagePreviousFrame = inImageCurrentFrame;
        }

        {
            auto blockEvent = this->inputBlockEvent();
            // Current frame
            inImageCurrentFrame = inputImage.get<ImgFrame>();
        }

        // Validate every frame before initializing or using the hardware accelerator dispatchers.
        const auto width = inImageCurrentFrame->getWidth();
        const auto height = inImageCurrentFrame->getHeight();
        if(width <= 0 || height <= 0 || width > 1920 || height > 1080 || width % 128 != 0 || height % 32 != 0) {
            logger->error(
                "Invalid input image dimensions {}x{} for FeatureTracker on RVC4. Width must be a positive multiple of 128 and at most 1920; "
                "height must be a positive multiple of 32 and at most 1080. Stopping the node.",
                width,
                height);
            stop();
            break;
        }

        // The dispatchers and their buffers are configured for the first frame's size.
        if(inImagePreviousFrame && (width != inImagePreviousFrame->getWidth() || height != inImagePreviousFrame->getHeight())) {
            logger->error("Input image dimensions changed from {}x{} to {}x{} for FeatureTracker on RVC4. "
                          "Input dimensions must remain constant. Stopping the node.",
                          inImagePreviousFrame->getWidth(),
                          inImagePreviousFrame->getHeight(),
                          width,
                          height);
            stop();
            break;
        }

        if(latestConfig.motionEstimator.enable == true) {
            if(!initedFpx) {
                cornerDetector.Open();
                cornerDetector.Init(inImageCurrentFrame, runAsync);
                cornerDetector.Config();
                initedFpx = true;
            }

            if(!initedOf) {
                opticalFlow.Open();
                opticalFlow.Init(inImageCurrentFrame, runAsync);
                opticalFlow.Config();
                initedOf = true;
            }
        } else {
            if(!initedFpx) {
                cornerDetector.Open();
                cornerDetector.Init(inImageCurrentFrame, runAsync);
                cornerDetector.Config();
                initedFpx = true;
            }
            if(!initedDcm) {
                descriptorCalculator.Open();
                descriptorCalculator.Init(inImageCurrentFrame, runAsync);
                descriptorCalculator.Config();
                initedDcm = true;
            }
        }

        auto tAbsoluteBeginning = steady_clock::now();
        auto tGotInput = tAbsoluteBeginning;

        // This is Optical Flow mode
        if(latestConfig.motionEstimator.enable == true) {
            std::map<TrackedPoint, uint32_t> motionTrackedFeaturesInCurrentFrame;

            // For the first frame, find feature points which will be tracked through Dense Optical Flow
            if(frameCounter == 1) {
                cornerDetector.Run(inImageCurrentFrame);
            } else {
                // Find all feature points in previous frame and call this set: "Tracked Features"
                cornerDetector.Run(inImagePreviousFrame);

                // Run Dense optical flow to calculate positions of previous frame's Tracked Features in current frame
                opticalFlow.Run(inImagePreviousFrame, inImageCurrentFrame);
            }

            // Add more features to try to follow
            // cornerDetector is holding detected points from previous frame
            // featureId is last id (it is simply incremented for every newly added feature)
            if(frameCounter % detectInterval == 1 or addMoreFeatures) {
                addMoreFeaturesToPreviousFrame(motionTrackedFeaturesInPreviousFrame,
                                               cornerDetector,
                                               minumumHarrisScore,
                                               featureId,
                                               logger,
                                               latestConfig.cornerDetector.numMaxFeatures,
                                               latestConfig.featureMaintainer.minimumDistanceBetweenFeatures);
            }

            // Run corner detection on current frame
            cornerDetector.Run(inImageCurrentFrame);

            // Only accept features that Dense Optical flow has matched in previous and current frame
            motionTrackedFeaturesInCurrentFrame = reidentifyUsingDenseOpticalFlow(motionTrackedFeaturesInPreviousFrame,
                                                                                  opticalFlow,
                                                                                  cornerDetector,
                                                                                  inImageCurrentFrame->getWidth(),
                                                                                  inImageCurrentFrame->getHeight(),
                                                                                  logger,
                                                                                  latestConfig.cornerDetector.numMaxFeatures,
                                                                                  100);

            extractFeatures(motionTrackedFeaturesInCurrentFrame, outFeatures);

            if(outFeatures->trackedFeatures.size() <= minimumNumberOfFeatures) {
                addMoreFeatures = true;
            } else {
                addMoreFeatures = false;
            }

            motionTrackedFeaturesInPreviousFrame = std::move(motionTrackedFeaturesInCurrentFrame);
        } else {
            // For each frame use all features as new features and calculate ORB descriptors
            cornerDetector.Run(inImageCurrentFrame);

            detectFeaturesInCurrentFrame(cornerDetector, minumumHarrisScore, featureId, logger, latestConfig.cornerDetector.numMaxFeatures, outFeatures);

            // if(latestConfig.cornerDetector.enableDescriptorCalculation && outFeatures->trackedFeatures.size() > 0) {
            if(true && outFeatures->trackedFeatures.size() > 0) {  // TODO(Morato) - expose
                calculateOrbDescriptors(inImageCurrentFrame, descriptorCalculator, outFeatures);
            }
        }

        bool hasConfig = false;

        if(inputConfig.getWaitForMessage()) {
            logger->trace("Receiving FeatureTracker config message!");
            inConfig = inputConfig.get<FeatureTrackerConfig>();
            hasConfig = true;
        } else {
            inConfig = inputConfig.tryGet<FeatureTrackerConfig>();
            if(inConfig != nullptr) {
                hasConfig = true;
            }
        }

        if(hasConfig) {
            bool alreadyMotionEnabled = latestConfig.motionEstimator.enable;

            latestConfig = *inConfig;

            if(latestConfig.cornerDetector.numMaxFeatures <= 0) latestConfig.cornerDetector.numMaxFeatures = defaultNumMaxFeatures;

            if(alreadyMotionEnabled && latestConfig.motionEstimator.enable == false) {
                // Purge data
                motionTrackedFeaturesInPreviousFrame.clear();
            }

            printConfig(latestConfig, logger);

            if(latestConfig.cornerDetector.thresholds.initialValue) cornerDetector.setThreshold(latestConfig.cornerDetector.thresholds.initialValue);
            cornerDetector.setRobustness(90);  // TODO(Morato) expose

            // Reconfigure the corner detector
            cornerDetector.Config();

            // Reconfigure the descriptor calculator
            descriptorCalculator.Config();
        }

        logger->trace("Number of features is: {}", outFeatures->trackedFeatures.size());
        logger->trace("This is a frame number: {}", frameCounter);

        outFeatures->setBufferMetadataFrom(inImageCurrentFrame);

        auto tProcessed = steady_clock::now();

        {
            auto blockEvent = this->outputBlockEvent();
            outputFeatures.send(outFeatures);
            passthroughInputImage.send(inImageCurrentFrame);
        }
        auto tAbsoluteEnd = steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);

        frameCounter++;
    }

    cornerDetector.Close();
    if(latestConfig.motionEstimator.enable == true) {
        opticalFlow.Close();
    } else {
        descriptorCalculator.Close();
    }
}
}  // namespace gate
}  // namespace dai
