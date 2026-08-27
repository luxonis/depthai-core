#include "depthai/pipeline/node/CropConfigGenerator.hpp"

#include <vector>

#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {
namespace {

template <typename Detection>
RotatedRect getDetectionBoundingBox(const Detection& detection) {
    if(detection.boundingBox.has_value()) {
        return *detection.boundingBox;
    }

    // ImgDetections legacy coordinates are defined as normalized coordinates.
    const auto width = detection.xmax - detection.xmin;
    const auto height = detection.ymax - detection.ymin;
    return RotatedRect(Point2f(detection.xmin + width / 2.0F, detection.ymin + height / 2.0F, true), Size2f(width, height, true), 0.0F);
}

template <typename Detections>
std::vector<RotatedRect> getBoundingBoxes(const Detections& detections) {
    std::vector<RotatedRect> boxes;
    boxes.reserve(detections.size());
    for(const auto& detection : detections) {
        boxes.push_back(getDetectionBoundingBox(detection));
    }
    return boxes;
}

std::vector<RotatedRect> getTrackletBoundingBoxes(const std::vector<Tracklet>& tracklets) {
    std::vector<RotatedRect> boxes;
    boxes.reserve(tracklets.size());
    for(const auto& tracklet : tracklets) {
        boxes.emplace_back(tracklet.roi);
    }
    return boxes;
}

template <typename Message, typename Extract>
std::vector<RotatedRect> transformAndExtract(const Message& message, const ImgFrame& image, Extract extract) {
    if(message.transformation.has_value() && message.transformation->isValid() && image.transformation.isValid()) {
        auto transformed = message;
        return extract(transformed.transformTo(image.transformation));
    }
    return extract(message);
}

}  // namespace

void CropConfigGenerator::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool CropConfigGenerator::runOnHost() const {
    return runOnHostVar;
}

void CropConfigGenerator::run() {
    auto& logger = pimpl->logger;

    while(mainLoop()) {
        std::shared_ptr<ADatatype> detections;
        std::shared_ptr<ImgFrame> image;
        {
            auto blockEvent = inputBlockEvent();
            detections = inputDetections.get();
            image = inputImage.get<ImgFrame>();
        }

        if(!detections || !image) {
            logger->warn("Invalid detection or image input, skipping pair");
            continue;
        }

        std::vector<RotatedRect> boxes;
        try {
            if(auto imgDetections = std::dynamic_pointer_cast<ImgDetections>(detections)) {
                boxes = transformAndExtract(*imgDetections, *image, [](const ImgDetections& message) { return getBoundingBoxes(message.detections); });
            } else if(auto spatialDetections = std::dynamic_pointer_cast<SpatialImgDetections>(detections)) {
                boxes =
                    transformAndExtract(*spatialDetections, *image, [](const SpatialImgDetections& message) { return getBoundingBoxes(message.detections); });
            } else if(auto tracklets = std::dynamic_pointer_cast<Tracklets>(detections)) {
                boxes = transformAndExtract(*tracklets, *image, [](const Tracklets& message) { return getTrackletBoundingBoxes(message.tracklets); });
            } else {
                logger->warn("Unsupported detection message type, skipping pair");
                continue;
            }
        } catch(const std::exception& ex) {
            logger->warn("Failed to map detection regions to input image: {}", ex.what());
            continue;
        }

        {
            auto blockEvent = outputBlockEvent();
            for(const auto& box : boxes) {
                if(box.size.width <= 0.0F || box.size.height <= 0.0F) {
                    logger->warn("Skipping detection with an empty bounding box");
                    continue;
                }

                auto config = std::make_shared<ImageManipConfig>();
                config->setBufferMetadataFrom(image);
                config->addCropRotatedRect(box, box.isNormalized());

                // Sending the config first preserves FIFO pairing when these outputs
                // are linked to ImageManip's inputConfig and inputImage.
                outConfig.send(config);
                outImage.send(image);
            }
        }
    }
}

}  // namespace node
}  // namespace dai
