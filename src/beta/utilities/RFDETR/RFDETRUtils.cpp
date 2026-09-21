#include "beta/utilities/RFDETR/RFDETRUtils.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "beta/utilities/Detection/MaskUtils.hpp"
#include "beta/utilities/MLSD/MLSDUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace RFDETRUtils {

namespace {

/// The segmentation mask encodes the instance index in a uint8 pixel with 255 as background,
/// so at most 255 instances (indices 0 to 254) fit, mirroring the source parser.
constexpr std::size_t MAX_SEGMENTATION_INSTANCES = 255;
constexpr std::uint8_t BACKGROUND = 255;

/// Tensor shape with all singleton dimensions removed, matching numpy squeeze().
std::vector<std::size_t> squeezedDims(const std::vector<std::size_t>& dims) {
    std::vector<std::size_t> result;
    for(const auto dim : dims) {
        if(dim != 1) {
            result.push_back(dim);
        }
    }
    return result;
}

/// Clip to [0, 1], propagating NaN like np.clip.
float clip01(float value) {
    return std::min(std::max(value, 0.0f), 1.0f);
}

}  // namespace

RfDetrDetections computeRfDetrDetections(const ClassificationUtils::ShapedTensorData& boxesTensor,
                                         const ClassificationUtils::ShapedTensorData& logitsTensor,
                                         const std::optional<ClassificationUtils::ShapedTensorData>& masksTensor,
                                         float confThreshold,
                                         int maxDetections,
                                         const std::vector<std::string>& labelNames,
                                         float maskConf,
                                         const std::optional<std::pair<std::uint32_t, std::uint32_t>>& inputSize) {
    DAI_CHECK(maxDetections > 0, "Max detections must be greater than 0.");

    // The logits tensor must be (1, N, C): the source takes the maximum over axis 2 and
    // squeezes the batch dimension away.
    DAI_CHECK_V(logitsTensor.dims.size() == 3 && logitsTensor.dims[0] == 1 && logitsTensor.dims[2] >= 1,
                "RFDETRParser: expected the logits tensor to have shape (1, N, C), got shape ({}).",
                fmt::join(logitsTensor.dims, ", "));
    const std::size_t numQueries = logitsTensor.dims[1];
    const std::size_t numClasses = logitsTensor.dims[2];

    // The boxes tensor must squeeze to (N, 4), matching the source boxes_tensor.squeeze().
    const auto boxesDims = squeezedDims(boxesTensor.dims);
    const bool boxesShapeValid =
        (boxesDims.size() == 2 && boxesDims[0] == numQueries && boxesDims[1] == 4) || (numQueries == 1 && boxesDims.size() == 1 && boxesDims[0] == 4);
    DAI_CHECK_V(
        boxesShapeValid, "RFDETRParser: expected the boxes tensor to squeeze to shape ({}, 4), got shape ({}).", numQueries, fmt::join(boxesTensor.dims, ", "));

    // The masks tensor, when present, must squeeze to (N, maskH, maskW).
    std::size_t maskLogitsHeight = 0;
    std::size_t maskLogitsWidth = 0;
    if(masksTensor.has_value()) {
        const auto masksDims = squeezedDims(masksTensor->dims);
        const bool masksShapeValid = (masksDims.size() == 3 && masksDims[0] == numQueries) || (numQueries == 1 && masksDims.size() == 2);
        DAI_CHECK_V(masksShapeValid,
                    "RFDETRParser: expected the masks tensor to squeeze to shape ({}, maskHeight, maskWidth), got shape ({}).",
                    numQueries,
                    fmt::join(masksTensor->dims, ", "));
        maskLogitsHeight = masksDims[masksDims.size() - 2];
        maskLogitsWidth = masksDims[masksDims.size() - 1];
    }

    // prob = sigmoid(logits); scores = max over the classes, labels = argmax. The maximum and
    // its first index follow numpy semantics: an update on !(value <= currentMax) propagates
    // NaN as maximal and returns the first index of the maximum.
    std::vector<float> scores(numQueries);
    std::vector<std::size_t> labels(numQueries);
    for(std::size_t query = 0; query < numQueries; ++query) {
        const float* queryLogits = logitsTensor.values.data() + query * numClasses;
        float maxProbability = MaskUtils::sigmoid(queryLogits[0]);
        std::size_t maxIndex = 0;
        if(!std::isnan(maxProbability)) {
            for(std::size_t classIndex = 1; classIndex < numClasses; ++classIndex) {
                const float probability = MaskUtils::sigmoid(queryLogits[classIndex]);
                if(!(probability <= maxProbability)) {
                    maxProbability = probability;
                    maxIndex = classIndex;
                    if(std::isnan(probability)) {
                        break;
                    }
                }
            }
        }
        scores[query] = maxProbability;
        labels[query] = maxIndex;
    }

    // sorted_idx = np.argsort(scores)[::-1], with numpy's generic argsort tie ordering.
    auto sortedIdx = MLSDUtils::argsortAscending(scores);
    std::reverse(sortedIdx.begin(), sortedIdx.end());

    // In segmentation mode the effective top-k is capped at the 255 instances the mask can
    // encode; confident instances beyond the cap are reported so the caller can warn.
    const auto maxDet = static_cast<std::size_t>(maxDetections);
    std::size_t effectiveMaxDet = maxDet;
    std::size_t ignoredInstances = 0;
    if(masksTensor.has_value()) {
        std::size_t numValidInstances = 0;
        for(std::size_t i = 0; i < std::min(maxDet, numQueries); ++i) {
            if(scores[sortedIdx[i]] > confThreshold) {
                numValidInstances++;
            }
        }
        if(numValidInstances > MAX_SEGMENTATION_INSTANCES) {
            ignoredInstances = numValidInstances - MAX_SEGMENTATION_INSTANCES;
        }
        effectiveMaxDet = std::min(effectiveMaxDet, MAX_SEGMENTATION_INSTANCES);
    }
    const std::size_t topK = std::min(effectiveMaxDet, numQueries);

    // Top-k slice in descending score order, then the strict greater-than confidence filter.
    // Kept per detection: the score, the label, the corner-form box clipped to [0, 1] and the
    // unclipped center-form box driving the mask crop.
    RfDetrDetections result;
    result.ignoredInstances = ignoredInstances;
    std::vector<std::array<float, 4>> keptBoxesXyxy;
    std::vector<std::array<float, 4>> keptBoxesCxcywh;
    std::vector<std::size_t> keptQueryIndices;
    for(std::size_t i = 0; i < topK; ++i) {
        const auto query = static_cast<std::size_t>(sortedIdx[i]);
        if(!(scores[query] > confThreshold)) {
            continue;
        }
        const float* box = boxesTensor.values.data() + query * 4;
        const std::array<float, 4> boxCxcywh = {box[0], box[1], box[2], box[3]};
        const std::array<float, 4> boxXyxy = {clip01(boxCxcywh[0] - boxCxcywh[2] / 2.0f),
                                              clip01(boxCxcywh[1] - boxCxcywh[3] / 2.0f),
                                              clip01(boxCxcywh[0] + boxCxcywh[2] / 2.0f),
                                              clip01(boxCxcywh[1] + boxCxcywh[3] / 2.0f)};
        result.scores.push_back(scores[query]);
        result.labels.push_back(static_cast<std::uint32_t>(labels[query]));
        keptBoxesXyxy.push_back(boxXyxy);
        keptBoxesCxcywh.push_back(boxCxcywh);
        keptQueryIndices.push_back(query);
    }

    // Segmentation mode: the final mask of the model input shape starts as background and each
    // detection claims its still-background pixels with the detection index, in descending
    // score order.
    if(masksTensor.has_value()) {
        DAI_CHECK(inputSize.has_value(), "RFDETRParser segmentation mode requires model input shape.");
        const auto outputWidth = static_cast<std::size_t>(inputSize->first);
        const auto outputHeight = static_cast<std::size_t>(inputSize->second);
        result.maskWidth = outputWidth;
        result.maskHeight = outputHeight;
        result.segmentationMask.assign(outputWidth * outputHeight, BACKGROUND);
        for(std::size_t i = 0; i < keptQueryIndices.size(); ++i) {
            const std::size_t maskSize = maskLogitsHeight * maskLogitsWidth;
            const span<const float> maskLogits(masksTensor->values.data() + keptQueryIndices[i] * maskSize, maskSize);
            const auto resizedMask =
                MaskUtils::processSingleMaskRfdetr(maskLogits, maskLogitsHeight, maskLogitsWidth, maskConf, keptBoxesCxcywh[i], outputHeight, outputWidth);
            for(std::size_t pixel = 0; pixel < result.segmentationMask.size(); ++pixel) {
                if(result.segmentationMask[pixel] == BACKGROUND && resizedMask[pixel] > 0) {
                    result.segmentationMask[pixel] = static_cast<std::uint8_t>(i);
                }
            }
        }
    }

    // Corner-form boxes back to center form, validating the corner ordering like the source
    // xyxy_to_xywh() (a NaN coordinate fails the validation as well).
    for(const auto& box : keptBoxesXyxy) {
        DAI_CHECK(box[0] <= box[2] && box[1] <= box[3], "Bounding box coordinates must be in the format [x_min, y_min, x_max, y_max].");
    }
    result.bboxes.reserve(keptBoxesXyxy.size());
    for(const auto& box : keptBoxesXyxy) {
        result.bboxes.push_back({(box[0] + box[2]) / 2.0f, (box[1] + box[3]) / 2.0f, box[2] - box[0], box[3] - box[1]});
    }

    // label_names[label] with a "class_<label>" fallback when the label is out of range.
    if(!labelNames.empty()) {
        result.labelNames.reserve(result.labels.size());
        for(const auto label : result.labels) {
            if(static_cast<std::size_t>(label) < labelNames.size()) {
                result.labelNames.push_back(labelNames[label]);
            } else {
                result.labelNames.push_back(fmt::format("class_{}", label));
            }
        }
    }

    return result;
}

}  // namespace RFDETRUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
