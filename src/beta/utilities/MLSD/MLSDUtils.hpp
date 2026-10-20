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

/**
 * @brief Decoded M-LSD line segments with their confidence scores.
 */
struct MlsdLines {
    /// Line segments as (startX, startY, endX, endY), normalized to the model input size.
    std::vector<std::array<float, 4>> lines;
    /// Confidence scores of the lines, index-aligned with lines.
    std::vector<float> scores;
};

/**
 * @brief Decode M-LSD center/displacement and heat map outputs into line segments.
 *
 * The tpMap tensor must be 4D in NCHW orientation with at least 5 channels; channel 0 (the
 * center map) is unused and channels 1 to 4 hold the (startDx, startDy, endDx, endDy)
 * displacement maps over the (height, width) grid of the first batch entry. The flattened heat
 * map is ranked and the topK highest-scoring positions are kept, ordered by descending score;
 * ties follow numpy's argpartition/argsort introselect/introsort semantics. Each kept position
 * (y, x) produces a candidate line from (x, y) plus its displacements; candidates are kept when
 * their score is strictly above scoreThreshold and their Euclidean length is strictly above
 * distanceThreshold. Kept line coordinates are scaled by 2 (the heat map grid is half the model
 * input resolution) and normalized by the model input size, x coordinates by inputWidth and y
 * coordinates by inputHeight.
 *
 * @param tpMapValues tpMap tensor values in row-major NCHW order.
 * @param tpMapDims tpMap tensor shape, must be (batch, channels >= 5, height, width).
 * @param heatValues Flattened heat map values; positions map to the tpMap grid row-major.
 * @param topK Number of top candidates to keep, capped at the heat map size. Must be positive.
 * @param scoreThreshold Confidence score threshold for detected lines (exclusive).
 * @param distanceThreshold Line length threshold in heat map grid units (exclusive).
 * @param inputWidth Model input image width the x coordinates are normalized by.
 * @param inputHeight Model input image height the y coordinates are normalized by.
 * @return Detected lines and their confidence scores, ordered by descending score.
 */
MlsdLines computeMlsdLines(const std::vector<float>& tpMapValues,
                           const std::vector<std::size_t>& tpMapDims,
                           const std::vector<float>& heatValues,
                           int topK,
                           float scoreThreshold,
                           float distanceThreshold,
                           std::uint32_t inputWidth,
                           std::uint32_t inputHeight);

/**
 * @brief Create a Lines message from decoded M-LSD lines.
 *
 * Validates that the number of scores matches the number of lines and that every confidence is
 * within [-0.1, 1.1]; confidences outside [0, 1] are clipped to [0, 1] with an info log,
 * matching the source message validation.
 *
 * @param mlsdLines Decoded lines with their confidence scores.
 * @return Lines message preserving the input order.
 */
std::shared_ptr<Lines> createLinesMessage(const MlsdLines& mlsdLines);

/**
 * @brief Return the indices of the topK highest values of the flattened heat map, ordered by
 * descending value.
 *
 * Replicates numpy's np.argpartition(heat, -topK)[-topK:] followed by
 * indices[np.argsort(-heat[indices])], including the generic (portable scalar) introselect and
 * introsort tie ordering. topK is capped at the heat size.
 *
 * @param heatValues Flattened heat map values.
 * @param topK Number of top candidates to keep. Must be positive.
 * @return Indices into heatValues ordered by descending value.
 */
std::vector<std::int64_t> topKIndicesByScore(const std::vector<float>& heatValues, int topK);

/**
 * @brief Return the indices that sort the values in descending order.
 *
 * Replicates numpy's np.argsort(-values), including the generic (portable scalar) introsort
 * tie ordering. An empty input yields an empty result.
 *
 * @param values Values to sort.
 * @return Indices into values ordered by descending value.
 */
std::vector<std::int64_t> argsortDescending(const std::vector<float>& values);

/**
 * @brief Return the indices that sort the values in ascending order.
 *
 * Replicates numpy's np.argsort(values), including the generic (portable scalar) introsort
 * tie ordering. An empty input yields an empty result. Reversing the result replicates
 * numpy's values.argsort()[::-1], whose tie ordering differs from np.argsort(-values).
 *
 * @param values Values to sort.
 * @return Indices into values ordered by ascending value.
 */
std::vector<std::int64_t> argsortAscending(const std::vector<float>& values);

}  // namespace MLSDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
