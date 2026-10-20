#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace XFeatUtils {

/**
 * @brief Dequantized FP32 tensor values in NCHW orientation together with the tensor shape.
 */
struct XFeatTensor {
    /// Tensor values in row-major order over dims.
    std::vector<float> values;
    /// Tensor shape in NCHW orientation.
    std::vector<std::size_t> dims;
};

/**
 * @brief The XFeat model output tensors of one frame in NCHW orientation.
 */
struct XFeatTensors {
    /// Dense feature (descriptor) map of shape (1, descriptor size, height, width).
    XFeatTensor feats;
    /// Keypoint logit map of shape (1, channels >= 64, height, width).
    XFeatTensor keypoints;
    /// Reliability heat map of shape (1, 1, height, width).
    XFeatTensor heatmaps;
};

/**
 * @brief Detected XFeat keypoints with their scores and descriptors.
 */
struct XFeatResult {
    /// Keypoint (x, y) positions scaled to the original image size, ordered by descending score.
    std::vector<std::array<float, 2>> keypoints;
    /// Keypoint scores, index-aligned with keypoints.
    std::vector<float> scores;
    /// L2-normalized keypoint descriptors, row-major (keypoints.size() x descriptorSize). The
    /// values are FP64 because the source bilinear() promotes the float32 sampling weights to
    /// float64 (int32 neighbor minus float32 position), making the emitted descriptors float64.
    std::vector<double> descriptors;
    /// Number of values in one descriptor (the feats tensor channel count).
    std::size_t descriptorSize = 0;
};

/**
 * @brief Matched keypoint positions between two XFeat results.
 */
struct MatchedPoints {
    /// Matched (x, y) positions from the reference result.
    std::vector<std::array<float, 2>> referencePoints;
    /// Matched (x, y) positions from the target result, index-aligned with referencePoints.
    std::vector<std::array<float, 2>> targetPoints;
};

/**
 * @brief Extract the feats, keypoints and heatmaps tensors from an XFeat NNData output.
 *
 * Each tensor is dequantized and permuted to NCHW orientation, matching the source parser's
 * output.getTensor(name, dequantize=True, storageOrder=NCHW). The tensor's stored order is
 * honored; the data is permuted, not just re-labeled.
 *
 * @param nnData NNData message containing the tensors.
 * @param featsName Name of the output layer containing the feature map.
 * @param keypointsName Name of the output layer containing the keypoint logits.
 * @param heatmapsName Name of the output layer containing the reliability heat map.
 * @return The three tensors in NCHW orientation.
 */
XFeatTensors extractXFeatTensors(dai::NNData& nnData, const std::string& featsName, const std::string& keypointsName, const std::string& heatmapsName);

/**
 * @brief Detect XFeat keypoints and compute their scores and descriptors, porting the source
 * detect_and_compute pipeline exactly.
 *
 * The feature map is L2-normalized along its channel axis. The keypoint logits are converted to
 * a per-cell softmax over all channels; the first 64 channels are unfolded into a full-resolution
 * (height * 8, width * 8) keypoint heat map. Candidate positions are local maxima of that map
 * (5x5 zero-padded window) with a value strictly above 0.05, in row-major scan order; when there
 * are none, std::nullopt is returned. Candidate scores are the product of the keypoint heat map
 * and the reliability heat map, both bilinearly sampled (align_corners=False grid convention,
 * normalized by the model input size) at the candidate positions; a candidate at position (0, 0)
 * gets score -1. Candidates are ordered by descending score with numpy argsort tie semantics and
 * the first topK are kept (negative topK follows Python slice semantics). Descriptors are
 * bilinearly sampled from the normalized feature map at the kept float32 positions (float64
 * weights and accumulation, following the numpy int32/float32 promotion) and L2-normalized.
 * Kept positions are scaled by the resize rates, and only candidates with a score strictly above
 * 0 are returned, so the result may hold zero keypoints.
 *
 * @param feats Feature map tensor of shape (1, descriptor size, height, width).
 * @param keypoints Keypoint logit tensor of shape (1, channels >= 64, height, width).
 * @param heatmaps Reliability heat map tensor of shape (1, 1, height, width).
 * @param resizeRateW Original-to-input width resize rate the x positions are scaled by.
 * @param resizeRateH Original-to-input height resize rate the y positions are scaled by.
 * @param inputWidth Model input image width used for the bilinear grid normalization.
 * @param inputHeight Model input image height used for the bilinear grid normalization.
 * @param topK Maximum number of keypoints to keep.
 * @return Detected keypoints with scores and descriptors, or std::nullopt when the keypoint heat
 *         map has no candidate above the threshold.
 */
std::optional<XFeatResult> detectAndCompute(const XFeatTensor& feats,
                                            const XFeatTensor& keypoints,
                                            const XFeatTensor& heatmaps,
                                            double resizeRateW,
                                            double resizeRateH,
                                            std::uint32_t inputWidth,
                                            std::uint32_t inputHeight,
                                            int topK);

/**
 * @brief Match the keypoints of two XFeat results by mutual nearest-neighbor cosine similarity,
 * porting the source match/_match_mkpts functions.
 *
 * The similarity of two descriptors is their float64 dot product (the descriptors are
 * L2-normalized). A reference keypoint i matches target keypoint j when j is the most similar
 * target for i and i is the most similar reference for j (first occurrence on ties, NaN
 * maximal, following numpy argmax). When minCossim is greater than 0, matches whose best
 * similarity is not strictly above minCossim are dropped. Matches are returned in ascending
 * reference-index order.
 *
 * Both results must hold at least one keypoint, matching the source behavior where numpy argmax
 * raises on an empty similarity axis.
 *
 * @param reference Reference (first) result.
 * @param target Target (second) result.
 * @param minCossim Minimum cosine similarity; values <= 0 disable the similarity filter. The
 *                  source parsers use the default -1 (mutual nearest neighbor only).
 * @return Matched reference and target keypoint positions.
 */
MatchedPoints matchResults(const XFeatResult& reference, const XFeatResult& target, double minCossim = -1.0);

/**
 * @brief Create a TrackedFeatures message from matched keypoint positions, porting the source
 * create_tracked_features_message creator.
 *
 * Validates that there are as many reference points as target points. Match i produces two
 * consecutive features with id i: the reference position with age 0 followed by the target
 * position with age 1. The remaining TrackedFeature fields keep their defaults.
 *
 * @param matchedPoints Matched reference and target keypoint positions.
 * @return TrackedFeatures message with the interleaved reference/target features.
 */
std::shared_ptr<dai::TrackedFeatures> createTrackedFeaturesMessage(const MatchedPoints& matchedPoints);

}  // namespace XFeatUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
