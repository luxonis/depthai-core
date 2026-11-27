#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Keypoints.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace KeypointsUtils {

struct KeypointCoordinates {
    std::vector<float> values;
    std::size_t numCoords = 0;  // 2 for (x, y), 3 for (x, y, z), 0 when empty

    std::size_t getNumKeypoints() const {
        return numCoords == 0 ? 0 : values.size() / numCoords;
    }
};

struct HrnetKeypoints {
    KeypointCoordinates coordinates;  // Normalized by the heatmap size
    std::vector<float> scores;        // Clipped to [0, 1]
};

struct SuperAnimalKeypoints {
    KeypointCoordinates coordinates;  // Normalized by the scale factor
    std::vector<float> scores;
};

KeypointCoordinates computeKeypoints(std::vector<float> values, std::int64_t nKeypoints, float scaleFactor);

HrnetKeypoints computeHrnetKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims);  // Row-major (numKeypoints, height, width) values

SuperAnimalKeypoints computeSuperAnimalKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims, float scaleFactor);

std::shared_ptr<Keypoints> createKeypointsMessage(const KeypointCoordinates& coordinates,  // Confidence -1.0 when no scores; z = 0 for 2D keypoints
                                                  const std::optional<std::vector<float>>& scores = std::nullopt,
                                                  std::optional<float> confidenceThreshold = std::nullopt,
                                                  const std::vector<std::string>& labelNames = {},
                                                  const std::vector<Edge>& edges = {});

std::optional<float> readFloatParam(const nlohmann::json& extraParams, const char* key);

std::optional<std::vector<std::string>> readKeypointLabelsParam(const nlohmann::json& extraParams);

std::vector<Edge> readSkeletonEdgesParam(const nlohmann::json& extraParams);  // Empty when the key is absent, null or an empty list

}  // namespace KeypointsUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
