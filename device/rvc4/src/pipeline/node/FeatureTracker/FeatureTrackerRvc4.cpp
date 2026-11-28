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
    constexpr std::int32_t defaultNumMaxFeatures = 512;  // From examples/python/FeatureTracker/feature_tracker.py: 256-1024 is a reasonable range
    if(latestConfig.cornerDetector.numMaxFeatures <= 0) latestConfig.cornerDetector.numMaxFeatures = defaultNumMaxFeatures;

    bool initedFpx = false;
    bool initedOf = false;
    bool initedDcm = false;

    bool runAsync = false;

    const uint32_t detectInterval = 1;

    while(mainLoop()) {
        std::shared_ptr<dai::TrackedFeatures> outFeatures = std::make_shared<dai::TrackedFeatures>();

        if(frameCounter > 1) {
            inImagePreviousFrame = inImageCurrentFrame;
        }

        {
            auto blockEvent = this->inputBlockEvent();
            inImageCurrentFrame = inputImage.get<ImgFrame>();
        }

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

        if(latestConfig.motionEstimator.enable == true) {
            std::map<TrackedPoint, uint32_t> motionTrackedFeaturesInCurrentFrame;

            if(frameCounter == 1) {
                cornerDetector.Run(inImageCurrentFrame);
            } else {
                cornerDetector.Run(inImagePreviousFrame);

                opticalFlow.Run(inImagePreviousFrame, inImageCurrentFrame);
            }

            if(frameCounter % detectInterval == 1 or addMoreFeatures) {
                addMoreFeaturesToPreviousFrame(motionTrackedFeaturesInPreviousFrame,
                                               cornerDetector,
                                               minumumHarrisScore,
                                               featureId,
                                               logger,
                                               latestConfig.cornerDetector.numMaxFeatures,
                                               latestConfig.featureMaintainer.minimumDistanceBetweenFeatures);
            }

            cornerDetector.Run(inImageCurrentFrame);

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
            cornerDetector.Run(inImageCurrentFrame);

            detectFeaturesInCurrentFrame(cornerDetector, minumumHarrisScore, featureId, logger, latestConfig.cornerDetector.numMaxFeatures, outFeatures);

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
                motionTrackedFeaturesInPreviousFrame.clear();
            }

            printConfig(latestConfig, logger);

            if(latestConfig.cornerDetector.thresholds.initialValue) cornerDetector.setThreshold(latestConfig.cornerDetector.thresholds.initialValue);
            cornerDetector.setRobustness(90);  // TODO(Morato) expose

            cornerDetector.Config();

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
