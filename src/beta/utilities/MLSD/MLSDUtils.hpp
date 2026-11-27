#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/beta/datatype/Lines.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace MLSDUtils {

struct MlsdLines {
    std::vector<std::array<float, 4>> lines;  // (startX, startY, endX, endY), normalized to the model input size
    std::vector<float> scores;
};

MlsdLines computeMlsdLines(const std::vector<float>& tpMapValues,  // distanceThreshold in heat map grid units; lines ordered by descending score
                           const std::vector<std::size_t>& tpMapDims,
                           const std::vector<float>& heatValues,
                           int topK,
                           float scoreThreshold,
                           float distanceThreshold,
                           std::uint32_t inputWidth,
                           std::uint32_t inputHeight);

std::shared_ptr<Lines> createLinesMessage(const MlsdLines& mlsdLines);  // Accepts confidences in [-0.1, 1.1] and clips them to [0, 1]

std::vector<std::int64_t> topKIndicesByScore(const std::vector<float>& heatValues, int topK);  // Same order as np.argpartition + np.argsort, ties included

std::vector<std::int64_t> argsortDescending(const std::vector<float>& values);

std::vector<std::int64_t> argsortAscending(const std::vector<float>& values);  // Reversed = values.argsort()[::-1], with other ties than np.argsort(-values)

}  // namespace MLSDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
