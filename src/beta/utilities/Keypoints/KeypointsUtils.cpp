#include "beta/utilities/Keypoints/KeypointsUtils.hpp"

#include <algorithm>
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
