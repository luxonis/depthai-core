#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace ClassificationUtils {

/**
 * @brief Dequantized FP32 tensor values together with the tensor shape.
 *
 * Values are stored contiguously in row-major order over dims, matching
 * NNData::getTensor(name, dequantize=true) without flattening.
 */
struct ShapedTensorData {
    /// Tensor values in row-major order over dims.
    std::vector<float> values;
    /// Tensor shape.
    std::vector<std::size_t> dims;
};

/**
 * @brief Per-step classification scores of shape (sequenceLength, nClasses), stored row-major.
 */
struct SequenceScores {
    /// Score values in row-major order, values.size() == sequenceLength * nClasses.
    std::vector<float> values;
    /// Number of sequence steps (rows).
    std::size_t sequenceLength = 0;
    /// Number of classes per step (columns).
    std::size_t nClasses = 0;
};

/**
 * @brief Compute the softmax of the given values: softmax(x) = exp(x) / sum(exp(x)).
 *
 * Uses the max-subtraction formulation for numerical stability; the result is
 * mathematically equivalent to the plain formulation.
 *
 * @param values Input values.
 * @return Softmax of the input values. Empty input yields an empty result.
 */
std::vector<float> softmax(const std::vector<float>& values);

/**
 * @brief Return classification scores, applying softmax when the raw scores are not already
 * softmaxed.
 *
 * @param scores Flattened raw scores.
 * @param isSoftmax True when the scores are already softmaxed; when false, softmax is applied.
 * @return Classification scores.
 */
std::vector<float> computeClassificationScores(std::vector<float> scores, bool isSoftmax);

/**
 * @brief Create a Classifications message with classes and scores sorted in descending order
 * of score (stable with respect to the input order on ties).
 *
 * Validates that classes and scores are non-empty, that every score is a probability in
 * [0, 1], that the scores sum to 1 (within an absolute tolerance of 0.1) and that the number
 * of classes matches the number of scores.
 *
 * @param classes Class names, index-aligned with scores.
 * @param scores Classification probability scores.
 * @return Classifications message with sorted classes and scores.
 */
std::shared_ptr<Classifications> createClassificationMessage(const std::vector<std::string>& classes, const std::vector<float>& scores);

/**
 * @brief Return per-step classification scores of shape (sequenceLength, nClasses), applying a
 * per-step softmax when the raw scores are not already softmaxed.
 *
 * Accepts a 2D tensor of shape (sequenceLength, nClasses) or a 3D tensor of shape
 * (1, sequenceLength, nClasses) or (sequenceLength, nClasses, 1); the singleton dimension is
 * squeezed. Any other rank or 3D shape raises an error.
 *
 * @param tensor Raw scores tensor with its shape.
 * @param isSoftmax True when the scores are already softmaxed; when false, softmax is applied
 *                  along each sequence step (row).
 * @return Per-step classification scores.
 */
SequenceScores computeClassificationSequenceScores(ShapedTensorData tensor, bool isSoftmax);

/**
 * @brief Create a Classifications message for a classification sequence. Classes and scores are
 * ordered by sequence position (never sorted by score).
 *
 * Per sequence step, the class with the maximum score is selected (first occurrence on ties).
 * When removeDuplicates is true, steps whose selected class equals the previous step's selected
 * class are dropped (consecutive duplicates only). Steps whose selected class index is listed in
 * ignoredIndexes are dropped. When concatenateClasses is true and more than one class remains:
 * when all remaining class names are at most one character long, they are joined and split on
 * whitespace into words with a per-word mean score; otherwise all class names are joined into a
 * single string with a " " separator and one mean score.
 *
 * Validates that the number of classes matches the number of score columns, that every score is
 * in [0, 1], that each row of scores sums to 1 (within an absolute tolerance of 0.01) and that
 * every ignored index is within [0, nClasses - 1]. An empty selection yields an empty message.
 *
 * @param classes Class names, index-aligned with the score columns.
 * @param scores Per-step classification probability scores.
 * @param ignoredIndexes Class indexes to drop from the sequence (e.g. background class, blank space).
 * @param removeDuplicates True to drop consecutive duplicate classes from the sequence.
 * @param concatenateClasses True to concatenate the remaining classes into words/a single string.
 * @return Classifications message ordered by sequence position.
 */
std::shared_ptr<Classifications> createClassificationSequenceMessage(const std::vector<std::string>& classes,
                                                                     const SequenceScores& scores,
                                                                     const std::vector<std::int32_t>& ignoredIndexes,
                                                                     bool removeDuplicates,
                                                                     bool concatenateClasses);

/**
 * @brief Retrieve a tensor from NNData as a flattened, dequantized FP32 vector.
 *
 * Elements are read in the tensor's native storage order and dequantized with the tensor's
 * quantization parameters when present, matching NNData::getTensor(name, dequantize=true)
 * followed by flattening.
 *
 * @param nnData NNData message containing the tensor.
 * @param tensorName Name of the tensor to retrieve.
 * @return Flattened dequantized tensor values.
 */
std::vector<float> getFlattenedTensorData(const dai::NNData& nnData, const std::string& tensorName);

/**
 * @brief Retrieve a tensor from NNData as a dequantized FP32 vector together with its shape.
 *
 * Elements are read in the tensor's native storage order and dequantized with the tensor's
 * quantization parameters when present, matching NNData::getTensor(name, dequantize=true) with
 * the shape preserved.
 *
 * @param nnData NNData message containing the tensor.
 * @param tensorName Name of the tensor to retrieve.
 * @return Dequantized tensor values with the tensor shape.
 */
ShapedTensorData getShapedTensorData(const dai::NNData& nnData, const std::string& tensorName);

}  // namespace ClassificationUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
