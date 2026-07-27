#include "depthai/beta/node/ImgDetectionsFilter.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace dai {
namespace beta {
namespace node {

namespace {

struct IndexedDetection {
    std::size_t originalIndex;
    ImgDetection detection;
};

bool containsLabel(const std::vector<std::uint32_t>& labels, std::uint32_t label) {
    return std::find(labels.begin(), labels.end(), label) != labels.end();
}

float detectionArea(const ImgDetection& detection) {
    const float width = std::max(0.0f, detection.xmax - detection.xmin);
    const float height = std::max(0.0f, detection.ymax - detection.ymin);
    return width * height;
}

float intersectionOverUnion(const ImgDetection& lhs, const ImgDetection& rhs) {
    const float intersectionWidth = std::max(0.0f, std::min(lhs.xmax, rhs.xmax) - std::max(lhs.xmin, rhs.xmin));
    const float intersectionHeight = std::max(0.0f, std::min(lhs.ymax, rhs.ymax) - std::max(lhs.ymin, rhs.ymin));
    const float intersectionArea = intersectionWidth * intersectionHeight;
    const float unionArea = detectionArea(lhs) + detectionArea(rhs) - intersectionArea;
    return unionArea > 0.0f ? intersectionArea / unionArea : 0.0f;
}

std::vector<IndexedDetection> filterDetections(const std::vector<ImgDetection>& detections, const ImgDetectionsFilterConfig& config) {
    std::vector<IndexedDetection> filtered;
    filtered.reserve(detections.size());

    for(std::size_t index = 0; index < detections.size(); ++index) {
        const auto& detection = detections[index];

        if(config.labelsToKeep.has_value() && !containsLabel(*config.labelsToKeep, detection.label)) {
            continue;
        }
        if(!config.labelsToKeep.has_value() && config.labelsToReject.has_value() && containsLabel(*config.labelsToReject, detection.label)) {
            continue;
        }
        if(config.confidenceThreshold.has_value() && detection.confidence < *config.confidenceThreshold) {
            continue;
        }
        if(config.minArea.has_value() && detectionArea(detection) < *config.minArea) {
            continue;
        }

        filtered.push_back({index, detection});
    }

    return filtered;
}

std::vector<IndexedDetection> applyNonMaximumSuppression(std::vector<IndexedDetection> detections, const ImgDetectionsFilterConfig& config) {
    detections.erase(std::remove_if(detections.begin(),
                                    detections.end(),
                                    [&config](const IndexedDetection& indexed) { return indexed.detection.confidence < config.nmsConfidenceThreshold; }),
                     detections.end());

    std::stable_sort(detections.begin(), detections.end(), [](const IndexedDetection& lhs, const IndexedDetection& rhs) {
        return lhs.detection.confidence > rhs.detection.confidence;
    });

    std::vector<IndexedDetection> kept;
    kept.reserve(detections.size());
    for(const auto& candidate : detections) {
        const bool suppressed = std::any_of(kept.begin(), kept.end(), [&candidate, &config](const IndexedDetection& selected) {
            return candidate.detection.label == selected.detection.label
                   && intersectionOverUnion(candidate.detection, selected.detection) > config.nmsIouThreshold;
        });
        if(!suppressed) {
            kept.push_back(candidate);
        }
    }

    return kept;
}

void remapSegmentationMask(const std::shared_ptr<ImgDetections>& output, const std::vector<IndexedDetection>& keptDetections) {
    auto mask = output->getMaskData();
    if(!mask.has_value()) {
        return;
    }

    std::array<std::uint8_t, 256> indexMap{};
    indexMap.fill(255);
    for(std::size_t newIndex = 0; newIndex < keptDetections.size() && newIndex < 255; ++newIndex) {
        const auto originalIndex = keptDetections[newIndex].originalIndex;
        if(originalIndex < 255) {
            indexMap[originalIndex] = static_cast<std::uint8_t>(newIndex);
        }
    }

    for(auto& instanceIndex : *mask) {
        instanceIndex = indexMap[instanceIndex];
    }
    output->setSegmentationMask(*mask, output->getSegmentationMaskWidth(), output->getSegmentationMaskHeight());
}

std::shared_ptr<ImgDetections> applyConfig(const std::shared_ptr<ImgDetections>& input, const ImgDetectionsFilterConfig& config) {
    if(config.isNoOp()) {
        return input;
    }

    auto filtered = filterDetections(input->detections, config);

    if(!config.nmsDisabled) {
        filtered = applyNonMaximumSuppression(std::move(filtered), config);
    }

    if(!config.sortingDisabled) {
        std::stable_sort(filtered.begin(), filtered.end(), [&config](const IndexedDetection& lhs, const IndexedDetection& rhs) {
            if(config.sortDescending) {
                return lhs.detection.confidence > rhs.detection.confidence;
            }
            return lhs.detection.confidence < rhs.detection.confidence;
        });
    }

    if(config.firstK.has_value() && filtered.size() > *config.firstK) {
        filtered.resize(*config.firstK);
    }

    auto output = std::make_shared<ImgDetections>(*input);
    output->detections.clear();
    output->detections.reserve(filtered.size());
    for(const auto& indexed : filtered) {
        output->detections.push_back(indexed.detection);
    }
    remapSegmentationMask(output, filtered);
    return output;
}

}  // namespace

ImgDetectionsFilter::~ImgDetectionsFilter() = default;

ImgDetectionsFilter::ImgDetectionsFilter(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ImgDetectionsFilter, ImgDetectionsFilterProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

void ImgDetectionsFilter::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool ImgDetectionsFilter::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void ImgDetectionsFilter::run() {
    auto config = getProperties().initialConfig;

    while(mainLoop()) {
        auto detections = input.get<ImgDetections>();
        if(!detections) {
            continue;
        }

        while(inputConfig.has()) {
            auto nextConfig = inputConfig.get<ImgDetectionsFilterConfig>();
            if(nextConfig) {
                config = *nextConfig;
            }
        }

        output.send(applyConfig(detections, config));
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
