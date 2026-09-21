#include "beta/utilities/Classification/ClassificationUtils.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>

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

// Tolerance matching np.isclose(rowSum, 1.0, atol=1e-2) with the default rtol of 1e-5.
constexpr double SEQUENCE_ROW_SUM_ATOL = 1e-2;
constexpr double SEQUENCE_ROW_SUM_RTOL = 1e-5;

// A single UTF-8 code point split into its byte sequence.
struct Utf8CodePoint {
    std::string bytes;
    bool isWhitespace = false;
};

// Whitespace set matching Python str.split() for ASCII whitespace characters.
bool isAsciiWhitespace(char character) {
    return character == ' ' || character == '\t' || character == '\n' || character == '\r' || character == '\v' || character == '\f';
}

// Split a UTF-8 string into code points. Continuation bytes (0b10xxxxxx) are grouped with their
// leading byte; invalid continuation bytes are treated as standalone code points.
std::vector<Utf8CodePoint> splitUtf8CodePoints(const std::string& text) {
    std::vector<Utf8CodePoint> codePoints;
    for(const char character : text) {
        const bool isContinuation = (static_cast<unsigned char>(character) & 0xC0) == 0x80;
        if(isContinuation && !codePoints.empty() && !codePoints.back().bytes.empty()
           && (static_cast<unsigned char>(codePoints.back().bytes.front()) & 0x80) != 0) {
            codePoints.back().bytes.push_back(character);
        } else {
            Utf8CodePoint codePoint;
            codePoint.bytes.push_back(character);
            codePoint.isWhitespace = isAsciiWhitespace(character);
            codePoints.push_back(std::move(codePoint));
        }
    }
    return codePoints;
}

// Number of UTF-8 code points in the string, matching Python len() for valid UTF-8.
std::size_t utf8Length(const std::string& text) {
    std::size_t length = 0;
    for(const char character : text) {
        if((static_cast<unsigned char>(character) & 0xC0) != 0x80) {
            ++length;
        }
    }
    return length;
}

// A word extracted from the joined class sequence together with its length in code points.
struct SequenceWord {
    std::string text;
    std::size_t codePointLength = 0;
};

// Split the joined class string on whitespace runs, matching Python str.split() with no
// arguments; leading, trailing and repeated whitespace produce no empty words.
std::vector<SequenceWord> splitOnWhitespace(const std::string& text) {
    std::vector<SequenceWord> words;
    SequenceWord current;
    for(const auto& codePoint : splitUtf8CodePoints(text)) {
        if(codePoint.isWhitespace) {
            if(current.codePointLength > 0) {
                words.push_back(std::move(current));
                current = SequenceWord{};
            }
        } else {
            current.text += codePoint.bytes;
            ++current.codePointLength;
        }
    }
    if(current.codePointLength > 0) {
        words.push_back(std::move(current));
    }
    return words;
}

// Mean of scores[begin, end), matching np.mean: an empty slice yields NaN.
float sliceMean(const std::vector<float>& scores, std::size_t begin, std::size_t end) {
    begin = std::min(begin, scores.size());
    end = std::min(end, scores.size());
    if(begin >= end) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    double sum = 0.0;
    for(std::size_t i = begin; i < end; ++i) {
        sum += static_cast<double>(scores[i]);
    }
    return static_cast<float>(sum / static_cast<double>(end - begin));
}

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

SequenceScores computeClassificationSequenceScores(ShapedTensorData tensor, bool isSoftmax) {
    auto& dims = tensor.dims;
    DAI_CHECK_V(dims.size() == 2 || dims.size() == 3, "Scores should be a 3D or 2D array, got shape ({}).", fmt::join(dims, ", "));

    if(dims.size() == 3) {
        if(dims[0] == 1) {
            // (1, sequence_length, n_classes) -> squeeze the batch dimension.
            dims.erase(dims.begin());
        } else if(dims[2] == 1) {
            // (sequence_length, n_classes, 1) -> squeeze the last dimension.
            dims.pop_back();
        } else {
            DAI_CHECK(false, "Scores should be a 3D array of shape (1, sequence_length, n_classes) or (sequence_length, n_classes, 1).");
        }
    }

    SequenceScores result;
    result.sequenceLength = dims[0];
    result.nClasses = dims[1];
    result.values = std::move(tensor.values);

    if(!isSoftmax) {
        // Softmax along axis 1 (per sequence step).
        std::vector<float> row(result.nClasses);
        for(std::size_t step = 0; step < result.sequenceLength; ++step) {
            const auto begin = result.values.begin() + static_cast<std::ptrdiff_t>(step * result.nClasses);
            std::copy(begin, begin + static_cast<std::ptrdiff_t>(result.nClasses), row.begin());
            const auto softmaxed = softmax(row);
            std::copy(softmaxed.begin(), softmaxed.end(), begin);
        }
    }

    return result;
}

std::shared_ptr<Classifications> createClassificationSequenceMessage(const std::vector<std::string>& classes,
                                                                     const SequenceScores& scores,
                                                                     const std::vector<std::int32_t>& ignoredIndexes,
                                                                     bool removeDuplicates,
                                                                     bool concatenateClasses) {
    const std::size_t sequenceLength = scores.sequenceLength;
    const std::size_t nClasses = scores.nClasses;
    DAI_CHECK_IN(scores.values.size() == sequenceLength * nClasses);

    DAI_CHECK_V(nClasses == classes.size(), "Number of classes and scores mismatch. Provided {} class names and {} scores.", classes.size(), nClasses);

    for(const auto value : scores.values) {
        DAI_CHECK(value >= 0.0f && value <= 1.0f, "Scores should be in the range [0, 1].");
    }

    for(std::size_t step = 0; step < sequenceLength; ++step) {
        double rowSum = 0.0;
        for(std::size_t classIndex = 0; classIndex < nClasses; ++classIndex) {
            rowSum += static_cast<double>(scores.values[step * nClasses + classIndex]);
        }
        DAI_CHECK_V(
            std::abs(rowSum - 1.0) <= SEQUENCE_ROW_SUM_ATOL + SEQUENCE_ROW_SUM_RTOL, "Each row of scores should sum to 1, got {} at row {}.", rowSum, step);
    }

    for(const auto ignoredIndex : ignoredIndexes) {
        DAI_CHECK(ignoredIndex >= 0 && static_cast<std::size_t>(ignoredIndex) < classes.size(),
                  "Ignored indexes should be integers in the range [0, num_classes -1].");
    }

    // Per-step argmax (first occurrence on ties) and row maximum.
    std::vector<std::size_t> indexes(sequenceLength, 0);
    std::vector<float> rowMax(sequenceLength, 0.0f);
    for(std::size_t step = 0; step < sequenceLength; ++step) {
        const auto begin = scores.values.begin() + static_cast<std::ptrdiff_t>(step * nClasses);
        const auto maxIt = std::max_element(begin, begin + static_cast<std::ptrdiff_t>(nClasses));
        indexes[step] = static_cast<std::size_t>(std::distance(begin, maxIt));
        rowMax[step] = *maxIt;
    }

    std::vector<bool> selection(sequenceLength, true);
    if(removeDuplicates) {
        for(std::size_t step = 1; step < sequenceLength; ++step) {
            selection[step] = indexes[step] != indexes[step - 1];
        }
    }
    for(std::size_t step = 0; step < sequenceLength; ++step) {
        if(selection[step]) {
            const bool ignored = std::find(ignoredIndexes.begin(), ignoredIndexes.end(), static_cast<std::int32_t>(indexes[step])) != ignoredIndexes.end();
            selection[step] = !ignored;
        }
    }

    // Selected classes and scores in sequence order (never sorted).
    std::vector<std::string> classList;
    std::vector<float> scoreList;
    for(std::size_t step = 0; step < sequenceLength; ++step) {
        if(selection[step]) {
            classList.push_back(classes[indexes[step]]);
            scoreList.push_back(rowMax[step]);
        }
    }

    if(concatenateClasses && classList.size() > 1) {
        const bool allSingleCharacter = std::all_of(classList.begin(), classList.end(), [](const std::string& word) { return utf8Length(word) <= 1; });
        if(allSingleCharacter) {
            // Join the single-character classes and split into words on whitespace. The score of
            // each word is the mean of the scores of its characters, offset by the number of
            // preceding whitespace separators.
            std::string joined;
            for(const auto& character : classList) {
                joined += character;
            }
            const auto words = splitOnWhitespace(joined);

            std::vector<std::string> concatenatedWords;
            std::vector<float> concatenatedScores;
            std::size_t startIndex = 0;
            std::size_t cumulativeLength = 0;
            for(std::size_t numSpaces = 0; numSpaces < words.size(); ++numSpaces) {
                cumulativeLength += words[numSpaces].codePointLength;
                const std::size_t endIndex = cumulativeLength;
                concatenatedWords.push_back(words[numSpaces].text);
                concatenatedScores.push_back(sliceMean(scoreList, startIndex + numSpaces, endIndex + numSpaces));
                startIndex = endIndex;
            }

            classList = std::move(concatenatedWords);
            scoreList = std::move(concatenatedScores);
        } else {
            // At least one class name is longer than one character: join everything into a single
            // string with a " " separator and one mean score.
            std::string joined;
            for(std::size_t i = 0; i < classList.size(); ++i) {
                if(i > 0) {
                    joined += " ";
                }
                joined += classList[i];
            }
            const float meanScore = sliceMean(scoreList, 0, scoreList.size());
            classList = {joined};
            scoreList = {meanScore};
        }
    }

    auto message = std::make_shared<Classifications>();
    message->classes = std::move(classList);
    message->scores = std::move(scoreList);
    return message;
}

std::vector<float> getFlattenedTensorData(const dai::NNData& nnData, const std::string& tensorName) {
    return getShapedTensorData(nnData, tensorName).values;
}

ShapedTensorData getShapedTensorData(const dai::NNData& nnData, const std::string& tensorName) {
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

    ShapedTensorData result;
    result.values = std::move(values);
    result.dims.assign(it->dims.begin(), it->dims.end());
    return result;
}

}  // namespace ClassificationUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
