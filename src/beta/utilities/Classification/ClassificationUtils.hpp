#pragma once

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

}  // namespace ClassificationUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
