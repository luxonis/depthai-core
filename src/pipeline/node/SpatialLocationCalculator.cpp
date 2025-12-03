#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "depthai/depthai.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/utilities/SpatialLocationCalculator/SpatialUtils.hpp"

namespace dai {
namespace node {

SpatialLocationCalculator::Properties& SpatialLocationCalculator::getProperties() {
    properties.roiConfig = *initialConfig;
    return properties;
}

void SpatialLocationCalculator::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool SpatialLocationCalculator::runOnHost() const {
    return runOnHostVar;
}

void SpatialLocationCalculator::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->warn("Start SpatialLocationCalculator");

    bool inputConfigSync = inputConfig.getWaitForMessage();
    auto eepromData = getParentPipeline().getEepromData();
    if(!eepromData) {
        logger->warn("No calibration data, using default FOV!");
    }

    while(isRunning()) {
        logger->warn("Running");
        bool hasConfig = false;
        std::shared_ptr<SpatialLocationCalculatorConfig> config;
        std::shared_ptr<ImgFrame> imgFrame;
        if(inputConfigSync) {
            config = inputConfig.get<dai::SpatialLocationCalculatorConfig>();
            if(config == nullptr) {
                logger->critical("Invalid input config.");
            } else {
                hasConfig = true;
            }
        } else {
            auto tmpConfig = inputConfig.tryGet<dai::SpatialLocationCalculatorConfig>();
            if(tmpConfig != nullptr) {
                config = tmpConfig;
                hasConfig = true;
            }
        }
        if(hasConfig) {
            roiConfig = config;
        }

        imgFrame = inputDepth.get<dai::ImgFrame>();
        if(imgFrame == nullptr) {
            logger->critical("Invalid input depth.");
        }
        if(imgFrame->getType() != dai::ImgFrame::Type::RAW16) {
            logger->critical("Invalid frame type for depth image. Depth image must be RAW16 type, got {}", static_cast<int>(imgFrame->getType()));
        }

        std::vector<SpatialLocations> spatialLocations;
        using namespace std::chrono;

        auto start = high_resolution_clock::now();
        auto outputSpatial = std::make_shared<SpatialLocationCalculatorData>();
        outputSpatial->setSequenceNum(imgFrame->getSequenceNum());
        outputSpatial->setTimestampDevice(imgFrame->getTimestampDevice());
        outputSpatial->setTimestamp(imgFrame->getTimestamp());
        if(roiConfig->getConfigData().size() > 0) {
            utilities::SpatialUtils::computeSpatialData(imgFrame, roiConfig->getConfigData(), spatialLocations, logger);
            auto stop = high_resolution_clock::now();
            auto timeToComputeSpatialData = duration_cast<microseconds>(stop - start);

            logger->trace("Time to compute spatial data CPU: {} us", timeToComputeSpatialData.count());
            outputSpatial->spatialLocations = std::move(spatialLocations);
        }

        // process imgDetections

        auto outputSpatialImgDetections = std::make_shared<dai::SpatialImgDetections>();
        if(input.isConnected()) {  // detections are connected and need to be processed
            std::shared_ptr<dai::ImgDetections> imgDetections = nullptr;
            imgDetections = input.get<dai::ImgDetections>();

            if(imgDetections != nullptr) {
                start = high_resolution_clock::now();
                utilities::SpatialUtils::computeSpatialDetections(imgFrame, roiConfig, *imgDetections, *outputSpatialImgDetections, logger);

                auto stop = high_resolution_clock::now();
                auto timeToComputeSpatialDetections = duration_cast<microseconds>(stop - start);
                logger->trace("Time to compute spatial detections: {} us", timeToComputeSpatialDetections.count());
            }
            outputSpatialImgDetections->setSequenceNum(imgFrame->getSequenceNum());
            outputSpatialImgDetections->setTimestampDevice(imgFrame->getTimestampDevice());
            outputSpatialImgDetections->setTimestamp(imgFrame->getTimestamp());
            outputSpatialImgDetections->transformation = imgFrame->transformation;
        }

        spatialOutput.send(outputSpatialImgDetections);
        out.send(outputSpatial);
        passthroughDepth.send(imgFrame);
    }
}
}  // namespace node
}  // namespace dai