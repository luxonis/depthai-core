#include "beta/utilities/Keypoints/KeypointsUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace KeypointsUtils {

KeypointCoordinates computeKeypoints(std::vector<float> values, std::int64_t nKeypoints, float scaleFactor) {
    DAI_CHECK(nKeypoints > 0, "Number of keypoints must be greater than 0.");

    // Matches int(np.prod(shape) / n_keypoints), which floors for non-divisible sizes.
    const std::size_t numCoords = values.size() / static_cast<std::size_t>(nKeypoints);
    DAI_CHECK_V(numCoords == 2 || numCoords == 3, "Expected 2 or 3 coordinates per keypoint, got {}.", numCoords);
    DAI_CHECK_V(values.size() == static_cast<std::size_t>(nKeypoints) * numCoords,
                "Cannot reshape {} keypoint values into shape ({}, {}).",
                values.size(),
                nKeypoints,
                numCoords);

    for(auto& value : values) {
        value = std::clamp(value / scaleFactor, 0.0f, 1.0f);
    }

    KeypointCoordinates coordinates;
    coordinates.values = std::move(values);
    coordinates.numCoords = numCoords;
    return coordinates;
}

HrnetKeypoints computeHrnetKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims) {
    std::vector<std::size_t> shape = dims;
    // A leading dimension of 1 (typically the batch dimension) is dropped exactly once; a batch
    // of more than 1 is not squeezed and fails the 3D check below, mirroring the source parser.
    if(!shape.empty() && shape.front() == 1) {
        shape.erase(shape.begin());
    }
    DAI_CHECK_V(shape.size() == 3, "Expected 3D output tensor, got {}D.", shape.size());

    const std::size_t numKeypoints = shape[0];
    const std::size_t mapHeight = shape[1];
    const std::size_t mapWidth = shape[2];
    const std::size_t mapSize = mapHeight * mapWidth;
    DAI_CHECK_V(numKeypoints > 0 && mapSize > 0, "Expected non-empty heatmaps, got shape ({}, {}, {}).", numKeypoints, mapHeight, mapWidth);
    DAI_CHECK_V(
        values.size() == numKeypoints * mapSize, "Cannot interpret {} heatmap values as shape ({}, {}, {}).", values.size(), numKeypoints, mapHeight, mapWidth);

    HrnetKeypoints result;
    result.coordinates.numCoords = 2;
    result.coordinates.values.reserve(numKeypoints * 2);
    result.scores.reserve(numKeypoints);

    for(std::size_t keypointIndex = 0; keypointIndex < numKeypoints; ++keypointIndex) {
        const float* heatmap = values.data() + keypointIndex * mapSize;
        // First maximum in row-major scan order, matching np.argmax: ties keep the first
        // occurrence and the first NaN is treated as the maximum.
        std::size_t maxIndex = 0;
        float maxValue = heatmap[0];
        for(std::size_t i = 1; i < mapSize && !std::isnan(maxValue); ++i) {
            if(std::isnan(heatmap[i]) || heatmap[i] > maxValue) {
                maxValue = heatmap[i];
                maxIndex = i;
            }
        }
        result.scores.push_back(std::clamp(maxValue, 0.0f, 1.0f));
        const std::size_t row = maxIndex / mapWidth;
        const std::size_t col = maxIndex % mapWidth;
        result.coordinates.values.push_back(static_cast<float>(col) / static_cast<float>(mapWidth));
        result.coordinates.values.push_back(static_cast<float>(row) / static_cast<float>(mapHeight));
    }
    return result;
}

SuperAnimalKeypoints computeSuperAnimalKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims, float scaleFactor) {
    DAI_CHECK_V(dims.size() == 4, "Expected a 4D heatmaps tensor of shape (batch, height, width, numKeypoints), got {}D.", dims.size());

    const std::size_t batchSize = dims[0];
    const std::size_t mapHeight = dims[1];
    const std::size_t mapWidth = dims[2];
    const std::size_t numKeypoints = dims[3];
    const std::size_t mapSize = mapHeight * mapWidth;
    // The source decode fails for any batch size other than 1 (the batch dimension is indexed at
    // 0 and the per-batch results cannot be recombined); reject it with an actionable error.
    DAI_CHECK_V(batchSize == 1, "Expected a heatmaps tensor with batch size 1, got batch size {}.", batchSize);
    DAI_CHECK_V(mapSize > 0, "Expected non-empty heatmaps, got shape ({}, {}, {}, {}).", batchSize, mapHeight, mapWidth, numKeypoints);
    DAI_CHECK_V(values.size() == mapSize * numKeypoints,
                "Cannot interpret {} heatmap values as shape ({}, {}, {}, {}).",
                values.size(),
                batchSize,
                mapHeight,
                mapWidth,
                numKeypoints);

    // Heatmap-cell size in input-image pixels, (scale_factor / height, scale_factor / width).
    const double scaleFactorY = static_cast<double>(scaleFactor) / static_cast<double>(mapHeight);
    const double scaleFactorX = static_cast<double>(scaleFactor) / static_cast<double>(mapWidth);

    SuperAnimalKeypoints result;
    result.coordinates.numCoords = 2;
    result.coordinates.values.reserve(numKeypoints * 2);
    result.scores.reserve(numKeypoints);

    for(std::size_t keypointIndex = 0; keypointIndex < numKeypoints; ++keypointIndex) {
        // First maximum over the flattened spatial dimension in row-major scan order, matching
        // np.argmax: ties keep the first occurrence and the first NaN is treated as the maximum.
        std::size_t maxIndex = 0;
        float maxValue = values[keypointIndex];
        for(std::size_t i = 1; i < mapSize && !std::isnan(maxValue); ++i) {
            const float value = values[i * numKeypoints + keypointIndex];
            if(std::isnan(value) || value > maxValue) {
                maxValue = value;
                maxIndex = i;
            }
        }
        // The score is the heatmap value at the maximum, intentionally not clipped.
        result.scores.push_back(maxValue);
        const std::size_t row = maxIndex / mapWidth;
        const std::size_t col = maxIndex % mapWidth;
        // Map to input-image pixels with a 0.5-pixel center offset, then normalize by the scale
        // factor. Computed in double precision to match the NumPy float64 arithmetic.
        const double x = static_cast<double>(col) * scaleFactorX + 0.5 * scaleFactorX;
        const double y = static_cast<double>(row) * scaleFactorY + 0.5 * scaleFactorY;
        result.coordinates.values.push_back(static_cast<float>(x / static_cast<double>(scaleFactor)));
        result.coordinates.values.push_back(static_cast<float>(y / static_cast<double>(scaleFactor)));
    }
    return result;
}

std::shared_ptr<Keypoints> createKeypointsMessage(const KeypointCoordinates& coordinates,
                                                  const std::optional<std::vector<float>>& scores,
                                                  std::optional<float> confidenceThreshold,
                                                  const std::vector<std::string>& labelNames,
                                                  const std::vector<Edge>& edges) {
    const std::size_t numKeypoints = coordinates.getNumKeypoints();
    if(numKeypoints != 0) {
        DAI_CHECK_V(
            coordinates.numCoords == 2 || coordinates.numCoords == 3, "All keypoints should be of dimension 2 or 3, got dimension {}.", coordinates.numCoords);
        DAI_CHECK_V(coordinates.values.size() == numKeypoints * coordinates.numCoords,
                    "Keypoints should be of shape (N,2 or 3) got ({} values, {} coordinates per keypoint).",
                    coordinates.values.size(),
                    coordinates.numCoords);
    }

    if(scores.has_value()) {
        DAI_CHECK_V(
            numKeypoints == scores->size(), "Keypoints and scores should have the same length. Got {} keypoints and {} scores.", numKeypoints, scores->size());
        for(const auto score : *scores) {
            DAI_CHECK(score >= 0.0f && score <= 1.0f, "Scores should only contain values between 0 and 1.");
        }
    }

    if(confidenceThreshold.has_value()) {
        DAI_CHECK_V(*confidenceThreshold >= 0.0f && *confidenceThreshold <= 1.0f,
                    "The confidence_threshold should be between 0 and 1, got confidence_threshold {}.",
                    *confidenceThreshold);
    }

    const bool use3D = coordinates.numCoords == 3;

    std::vector<Keypoint> points;
    points.reserve(numKeypoints);
    // Maps the original keypoint index to the index within the kept keypoints.
    std::unordered_map<std::uint32_t, std::uint32_t> indexMapping;
    indexMapping.reserve(numKeypoints);

    for(std::size_t i = 0; i < numKeypoints; ++i) {
        if(scores.has_value() && confidenceThreshold.has_value() && (*scores)[i] < *confidenceThreshold) {
            continue;
        }

        Keypoint keypoint;
        const std::size_t offset = i * coordinates.numCoords;
        keypoint.imageCoordinates = Point3f(coordinates.values[offset], coordinates.values[offset + 1], use3D ? coordinates.values[offset + 2] : 0.0f);
        keypoint.confidence = scores.has_value() ? (*scores)[i] : -1.0f;
        if(!labelNames.empty()) {
            DAI_CHECK_V(i < labelNames.size(), "Label names should cover all keypoints. Got {} label names and {} keypoints.", labelNames.size(), numKeypoints);
            keypoint.labelName = labelNames[i];
        }
        indexMapping.emplace(static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(points.size()));
        points.push_back(std::move(keypoint));
    }

    auto message = std::make_shared<Keypoints>();
    message->keypointsList.setKeypoints(points);

    if(!edges.empty()) {
        // Keep only edges whose both endpoints were kept and remap them to the new indices.
        std::vector<Edge> filteredEdges;
        filteredEdges.reserve(edges.size());
        for(const auto& edge : edges) {
            const auto first = indexMapping.find(edge[0]);
            const auto second = indexMapping.find(edge[1]);
            if(first != indexMapping.end() && second != indexMapping.end()) {
                filteredEdges.push_back(Edge{first->second, second->second});
            }
        }
        message->keypointsList.setEdges(filteredEdges);
    }

    return message;
}

}  // namespace KeypointsUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
