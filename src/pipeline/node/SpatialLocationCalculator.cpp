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

bool SpatialLocationCalculator::runOnHost() const {
    return runOnHostVar;
}

void SpatialLocationCalculator::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("Start SpatialLocationCalculator");

    bool inputConfigSync = inputConfig.getWaitForMessage();
    auto eepromData = getParentPipeline().getEepromData();
    if(!eepromData) {
        logger->warn("No calibration data, using default FOV!");
    }

    using namespace std::chrono;
    while(isRunning()) {
        auto tAbsoluteBeginning = steady_clock::now();

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
            calculationConfig = config;
        }

        imgFrame = inputDepth.get<dai::ImgFrame>();
        if(imgFrame == nullptr) {
            logger->critical("Invalid input depth.");
        }
        if(imgFrame->getType() != dai::ImgFrame::Type::RAW16) {
            logger->critical("Invalid frame type for depth image. Depth image must be RAW16 type, got {}", static_cast<int>(imgFrame->getType()));
        }
        auto tAfterMessageBeginning = steady_clock::now();

        std::vector<SpatialLocations> spatialLocations;

        auto start = high_resolution_clock::now();
        auto outputSpatial = std::make_shared<SpatialLocationCalculatorData>();
        outputSpatial->setSequenceNum(imgFrame->getSequenceNum());
        outputSpatial->setTimestampDevice(imgFrame->getTimestampDevice());
        outputSpatial->setTimestamp(imgFrame->getTimestamp());
        if(calculationConfig->getConfigData().size() > 0) {
            utilities::SpatialUtils::computeSpatialData(imgFrame, calculationConfig->getConfigData(), spatialLocations, logger);
            auto stop = high_resolution_clock::now();
            auto timeToComputeSpatialData = duration_cast<microseconds>(stop - start);

            logger->trace("Time to compute spatial data CPU: {} us", timeToComputeSpatialData.count());
            outputSpatial->spatialLocations = std::move(spatialLocations);
        }

        // process imgDetections

        auto outputSpatialImgDetections = std::make_shared<dai::SpatialImgDetections>();
        if(inputDetections.isConnected()) {  // detections are connected and need to be processed
            std::shared_ptr<dai::ImgDetections> imgDetections = nullptr;
            imgDetections = inputDetections.get<dai::ImgDetections>();

            if(imgDetections != nullptr) {
                start = high_resolution_clock::now();
                utilities::SpatialUtils::computeSpatialDetections(*imgFrame, *calculationConfig, *imgDetections, *outputSpatialImgDetections, logger);

                auto stop = high_resolution_clock::now();
                auto timeToComputeSpatialDetections = duration_cast<microseconds>(stop - start);
                logger->trace("Time to compute spatial detections: {} us", timeToComputeSpatialDetections.count());
            }
            outputSpatialImgDetections->setSequenceNum(imgDetections->getSequenceNum());
            outputSpatialImgDetections->setTimestampDevice(imgDetections->getTimestampDevice());
            outputSpatialImgDetections->setTimestamp(imgDetections->getTimestamp());
            outputSpatialImgDetections->transformation = imgDetections->transformation;
        }

        auto tBeforeSend = steady_clock::now();

        outputDetections.send(outputSpatialImgDetections);
        out.send(outputSpatial);
        passthroughDepth.send(imgFrame);

        auto tAbsoluteEnd = steady_clock::now();
        logger->debug("SpatialLocationCalculator total took {}ms, processing {}ms, getting_frames {}ms, sending_frames {}ms",
                      duration_cast<microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tBeforeSend - tAfterMessageBeginning).count() / 1000,
                      duration_cast<microseconds>(tAfterMessageBeginning - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tAbsoluteEnd - tBeforeSend).count() / 1000);
    }
}
}  // namespace node
}  // namespace dai