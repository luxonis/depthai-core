#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace ClassificationUtils {

struct ShapedTensorData {
    std::vector<float> values;
    std::vector<std::size_t> dims;
};

struct SequenceScores {
    std::vector<float> values;
    std::size_t sequenceLength = 0;
    std::size_t nClasses = 0;
};

std::vector<float> softmax(const std::vector<float>& values);

std::vector<float> computeClassificationScores(std::vector<float> scores, bool isSoftmax);

std::shared_ptr<Classifications> createClassificationMessage(const std::vector<std::string>& classes, const std::vector<float>& scores);

SequenceScores computeClassificationSequenceScores(ShapedTensorData tensor, bool isSoftmax);

std::shared_ptr<Classifications> createClassificationSequenceMessage(const std::vector<std::string>& classes,  // Ordered by sequence position, never by score
                                                                     const SequenceScores& scores,
                                                                     const std::vector<std::int32_t>& ignoredIndexes,
                                                                     bool removeDuplicates,
                                                                     bool concatenateClasses);

std::vector<float> getFlattenedTensorData(const dai::NNData& nnData, const std::string& tensorName);

ShapedTensorData getShapedTensorData(const dai::NNData& nnData, const std::string& tensorName, std::optional<TensorInfo::StorageOrder> order = std::nullopt);

ShapedTensorData getNchwTensorData(const dai::NNData& nnData, const std::string& tensorName);  // Dequantized FP32; data permuted to NCHW, not re-labeled

}  // namespace ClassificationUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
