// This file contains utility functions for decoding the output of the YuNet face detection
// model.
//
// This file contains code that is based on or directly taken from a public GitHub repository:
// https://github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample
//
// Original code author(s): Kazuhito00
//
// License: Apache License 2.0
//
// Copyright (c) [2021] [Kazuhito00]

#include "beta/utilities/YuNet/YuNetUtils.hpp"

#include <cmath>
#include <limits>

#include "beta/utilities/Detection/DetectionUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace YuNetUtils {

namespace {

using DetectionUtils::clip01;

inline float truncateToInt32(float value) {                 // NaN and out-of-range saturate to the int32 minimum, as numpy on x86 (cvttss2si)
    constexpr float INT32_MIN_F = -2147483648.0f;           // int32 minimum, exactly representable.
    constexpr float INT32_MAX_TRUNCATABLE = 2147483520.0f;  // Largest float32 below 2^31.
    if(!(value >= INT32_MIN_F && value <= INT32_MAX_TRUNCATABLE)) {
        return INT32_MIN_F;
    }
    return static_cast<float>(static_cast<std::int32_t>(value));
}

}  // namespace

std::vector<std::array<float, 4>> generateAnchors(std::uint32_t inputWidth, std::uint32_t inputHeight) {
    DAI_CHECK(inputWidth > 0 && inputHeight > 0, "Input size must be greater than 0.");

    const std::vector<std::vector<std::int64_t>> minSizes{{10, 16, 24}, {32, 48}, {64, 96}, {128, 192, 256}};  // Fixed by the YuNet anchor contract
    const std::vector<std::int64_t> strides{8, 16, 32, 64};

    const std::int64_t w = static_cast<std::int64_t>(inputWidth);
    const std::int64_t h = static_cast<std::int64_t>(inputHeight);

    std::array<std::int64_t, 2> featureMap{((h + 1) / 2) / 2, ((w + 1) / 2) / 2};
    std::vector<std::array<std::int64_t, 2>> featureMaps;
    featureMaps.reserve(strides.size());
    for(std::size_t level = 0; level < strides.size(); level++) {
        featureMap = {featureMap[0] / 2, featureMap[1] / 2};
        featureMaps.push_back(featureMap);
    }

    std::vector<std::array<float, 4>> anchors;
    for(std::size_t k = 0; k < featureMaps.size(); k++) {
        const auto& f = featureMaps[k];
        for(std::int64_t i = 0; i < f[0]; i++) {
            for(std::int64_t j = 0; j < f[1]; j++) {
                for(const std::int64_t minSize : minSizes[k]) {
                    const float sKx = static_cast<float>(static_cast<double>(minSize) / static_cast<double>(w));
                    const float sKy = static_cast<float>(static_cast<double>(minSize) / static_cast<double>(h));
                    const float cx = static_cast<float>((static_cast<double>(j) + 0.5) * static_cast<double>(strides[k]) / static_cast<double>(w));
                    const float cy = static_cast<float>((static_cast<double>(i) + 0.5) * static_cast<double>(strides[k]) / static_cast<double>(h));
                    anchors.push_back({cx, cy, sKx, sKy});
                }
            }
        }
    }
    return anchors;
}

YuNetDetections computeYuNetDetections(const std::vector<float>& loc,
                                       const std::vector<float>& conf,
                                       const std::vector<float>& iou,
                                       const std::vector<std::array<float, 4>>& anchors,
                                       std::uint32_t inputWidth,
                                       std::uint32_t inputHeight,
                                       float confThreshold,
                                       float iouThreshold,
                                       int maxDetections) {
    const std::size_t numCandidates = iou.size() / IOU_VALUES_PER_ANCHOR;
    DAI_CHECK_V(loc.size() == numCandidates * LOC_VALUES_PER_ANCHOR,
                "Expected {} loc values for {} candidates, got {}.",
                numCandidates * LOC_VALUES_PER_ANCHOR,
                numCandidates,
                loc.size());
    DAI_CHECK_V(conf.size() == numCandidates * CONF_VALUES_PER_ANCHOR,
                "Expected {} conf values for {} candidates, got {}.",
                numCandidates * CONF_VALUES_PER_ANCHOR,
                numCandidates,
                conf.size());
    DAI_CHECK_V(anchors.size() == numCandidates,
                "Got {} candidates but {} anchors; the tensor sizes must match the anchor count derived from the input size ({}, {}).",
                numCandidates,
                anchors.size(),
                inputWidth,
                inputHeight);

    constexpr float VAR0 = 0.1f;
    constexpr float VAR1 = 0.2f;

    const float scaleX = static_cast<float>(inputWidth);
    const float scaleY = static_cast<float>(inputHeight);

    std::vector<std::array<float, 4>> topLeftBboxes;              // Normalized top-left (x, y, width, height).
    std::vector<std::array<double, 4>> nmsBboxes;                 // The same boxes widened to double for the NMS.
    std::vector<float> scores;                                    // Pruned candidate scores.
    std::vector<std::array<float, 2 * NUM_KEYPOINTS>> keypoints;  // Normalized keypoint coordinates.
    for(std::size_t n = 0; n < numCandidates; n++) {
        const float clsScore = conf[n * CONF_VALUES_PER_ANCHOR + 1];
        const float iouScore = clip01(iou[n * IOU_VALUES_PER_ANCHOR]);
        const float score = std::sqrt(clsScore * iouScore);
        if(!(score > confThreshold)) {
            continue;
        }

        const auto& anchor = anchors[n];
        const float* locRow = loc.data() + n * LOC_VALUES_PER_ANCHOR;

        const float centerX = (anchor[0] + locRow[0] * VAR0 * anchor[2]) * scaleX;
        const float centerY = (anchor[1] + locRow[1] * VAR0 * anchor[3]) * scaleY;
        const float width = (anchor[2] * std::exp(locRow[2] * VAR1)) * scaleX;
        const float height = (anchor[3] * std::exp(locRow[3] * VAR1)) * scaleY;
        const std::array<float, 4> box{(centerX - width / 2.0f) / scaleX, (centerY - height / 2.0f) / scaleY, width / scaleX, height / scaleY};
        topLeftBboxes.push_back(box);
        nmsBboxes.push_back({static_cast<double>(box[0]), static_cast<double>(box[1]), static_cast<double>(box[2]), static_cast<double>(box[3])});
        scores.push_back(score);

        std::array<float, 2 * NUM_KEYPOINTS> keypointRow{};  // Truncated to int32 like the source format_detections' astype(np.int32)
        for(std::size_t i = 0; i < NUM_KEYPOINTS; i++) {
            const float keypointX = (anchor[0] + locRow[4 + 2 * i] * VAR0 * anchor[2]) * scaleX;
            const float keypointY = (anchor[1] + locRow[5 + 2 * i] * VAR0 * anchor[3]) * scaleY;
            keypointRow[2 * i] = truncateToInt32(keypointX) / scaleX;
            keypointRow[2 * i + 1] = truncateToInt32(keypointY) / scaleY;
        }
        keypoints.push_back(keypointRow);
    }

    const std::vector<int> keep = DetectionUtils::nmsBoxes(nmsBboxes, scores, confThreshold, iouThreshold, maxDetections);

    YuNetDetections detections;
    detections.bboxes.reserve(keep.size());
    detections.scores.reserve(keep.size());
    detections.keypoints.reserve(keep.size());
    for(const int keptIdx : keep) {
        const auto& box = topLeftBboxes[keptIdx];
        detections.bboxes.push_back({clip01(box[0] + box[2] / 2.0f), clip01(box[1] + box[3] / 2.0f), clip01(box[2]), clip01(box[3])});
        detections.scores.push_back(scores[keptIdx]);

        const auto& keypointRow = keypoints[keptIdx];
        std::vector<std::array<float, 2>> detectionKeypoints(NUM_KEYPOINTS);
        for(std::size_t i = 0; i < NUM_KEYPOINTS; i++) {
            detectionKeypoints[i][0] = clip01(keypointRow[2 * i]);
            detectionKeypoints[i][1] = clip01(keypointRow[2 * i + 1]);
        }
        detections.keypoints.push_back(std::move(detectionKeypoints));
    }
    return detections;
}

}  // namespace YuNetUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
