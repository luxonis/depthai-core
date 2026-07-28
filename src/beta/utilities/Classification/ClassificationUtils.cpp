#include "beta/utilities/Classification/ClassificationUtils.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

#include "fp16/fp16.h"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace ClassificationUtils {

namespace {

// Tolerance matching np.isclose(sum, 1.0, atol=1e-1) with the default rtol of 1e-5.
constexpr double SCORE_SUM_ATOL = 1e-1;
constexpr double SCORE_SUM_RTOL = 1e-5;

}  // namespace

std::vector<float> softmax(const std::vector<float>& values) {
    if(values.empty()) {
        return {};
    }
    const float maxValue = *std::max_element(values.begin(), values.end());
    std::vector<float> result(values.size());
    double sum = 0.0;
    for(std::size_t i = 0; i < values.size(); ++i) {
        result[i] = std::exp(values[i] - maxValue);
        sum += static_cast<double>(result[i]);
    }
    for(auto& value : result) {
        value = static_cast<float>(static_cast<double>(value) / sum);
    }
    return result;
}

std::vector<float> computeClassificationScores(std::vector<float> scores, bool isSoftmax) {
    if(!isSoftmax) {
        return softmax(scores);
    }
    return scores;
}

std::shared_ptr<Classifications> createClassificationMessage(const std::vector<std::string>& classes, const std::vector<float>& scores) {
    DAI_CHECK(!classes.empty(), "Classes should not be empty.");
    DAI_CHECK(!scores.empty(), "Scores should not be empty.");

    for(std::size_t i = 0; i < scores.size(); ++i) {
        DAI_CHECK_V(scores[i] >= 0.0f && scores[i] <= 1.0f, "Scores must contain probabilities between 0 and 1, got {} at index {}.", scores[i], i);
    }

    const double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    DAI_CHECK_V(std::abs(sum - 1.0) <= SCORE_SUM_ATOL + SCORE_SUM_RTOL, "Scores should sum to 1, got {}.", sum);

    DAI_CHECK_V(scores.size() == classes.size(), "Number of labels and scores mismatch. Provided {} scores and {} class names.", scores.size(), classes.size());

    // Stable descending sort by score, matching np.argsort(-scores, kind="stable").
    std::vector<std::size_t> indices(scores.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::stable_sort(indices.begin(), indices.end(), [&scores](std::size_t lhs, std::size_t rhs) { return scores[lhs] > scores[rhs]; });

    auto message = std::make_shared<Classifications>();
    message->classes.reserve(indices.size());
    message->scores.reserve(indices.size());
    for(const auto index : indices) {
        message->classes.push_back(classes[index]);
        message->scores.push_back(scores[index]);
    }
    return message;
}

std::vector<float> getFlattenedTensorData(const dai::NNData& nnData, const std::string& tensorName) {
    const auto it = std::find_if(nnData.tensors.begin(), nnData.tensors.end(), [&tensorName](const TensorInfo& info) { return info.name == tensorName; });
    DAI_CHECK_V(it != nnData.tensors.end(), "Tensor '{}' does not exist in NNData.", tensorName);

    std::size_t elementCount = it->dims.empty() ? 0 : 1;
    for(const auto dim : it->dims) {
        elementCount *= dim;
    }

    const auto data = nnData.getData();
    const std::size_t requiredBytes = static_cast<std::size_t>(it->offset) + elementCount * static_cast<std::size_t>(it->getDataTypeSize());
    DAI_CHECK_V(requiredBytes <= data.size(),
                "Tensor '{}' data is out of bounds: requires {} bytes, NNData payload holds {} bytes.",
                tensorName,
                requiredBytes,
                data.size());

    std::vector<float> values(elementCount);
    const std::uint8_t* base = data.data();
    switch(it->dataType) {
        case TensorInfo::DataType::U8F:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = static_cast<float>(base[it->offset + i]);
            }
            break;
        case TensorInfo::DataType::I8:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = static_cast<float>(reinterpret_cast<const std::int8_t*>(base)[it->offset + i]);
            }
            break;
        case TensorInfo::DataType::U16F:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = static_cast<float>(reinterpret_cast<const std::uint16_t*>(base)[it->offset / sizeof(std::uint16_t) + i]);
            }
            break;
        case TensorInfo::DataType::INT:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = static_cast<float>(reinterpret_cast<const std::int32_t*>(base)[it->offset / sizeof(std::int32_t) + i]);
            }
            break;
        case TensorInfo::DataType::FP16:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = fp16_ieee_to_fp32_value(reinterpret_cast<const std::uint16_t*>(base)[it->offset / sizeof(std::uint16_t) + i]);
            }
            break;
        case TensorInfo::DataType::FP32:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = reinterpret_cast<const float*>(base)[it->offset / sizeof(float) + i];
            }
            break;
        case TensorInfo::DataType::FP64:
            for(std::size_t i = 0; i < elementCount; ++i) {
                values[i] = static_cast<float>(reinterpret_cast<const double*>(base)[it->offset / sizeof(double) + i]);
            }
            break;
    }

    if(it->quantization) {
        for(auto& value : values) {
            value = (value - it->qpZp) * it->qpScale;
        }
    }
    return values;
}

}  // namespace ClassificationUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
