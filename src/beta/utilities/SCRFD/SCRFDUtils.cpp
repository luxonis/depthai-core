#include "beta/utilities/SCRFD/SCRFDUtils.hpp"

#include <algorithm>
#include <utility>

#include "beta/utilities/MLSD/MLSDUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace SCRFDUtils {

namespace {

/// numpy np.maximum: NaN in either operand propagates, unlike std::max.
inline float npMaximum(float a, float b) {
    if(a != a) {
        return a;
    }
    if(b != b) {
        return b;
    }
    return a > b ? a : b;
}

/// numpy np.minimum: NaN in either operand propagates, unlike std::min.
inline float npMinimum(float a, float b) {
    if(a != a) {
        return a;
    }
    if(b != b) {
        return b;
    }
    return a < b ? a : b;
}

/// numpy np.clip(value, 0, 1) in double precision; NaN propagates.
inline double clip01(double value) {
    return std::min(std::max(value, 0.0), 1.0);
}

}  // namespace

std::vector<std::array<float, 2>> computeAnchorCenters(std::int64_t stride, std::uint32_t inputWidth, std::uint32_t inputHeight, std::int64_t numAnchors) {
    DAI_CHECK_V(stride > 0, "Feature stride must be positive, got {}.", stride);

    // height = input_height // stride, width = input_width // stride (integer division).
    const std::int64_t gridHeight = static_cast<std::int64_t>(inputHeight) / stride;
    const std::int64_t gridWidth = static_cast<std::int64_t>(inputWidth) / stride;

    // np.stack(np.mgrid[:height, :width][::-1], axis=-1) * stride: (x, y) = (column, row)
    // scaled by the stride, row-major over (row, column). When num_anchors > 1 every center is
    // duplicated num_anchors times consecutively (np.stack([anchor_centers] * num_anchors,
    // axis=1)); the source duplicates only when num_anchors > 1, so 1 or less yields the
    // unduplicated grid.
    const std::int64_t duplicates = numAnchors > 1 ? numAnchors : 1;
    std::vector<std::array<float, 2>> anchorCenters;
    anchorCenters.reserve(static_cast<std::size_t>(gridHeight * gridWidth * duplicates));
    for(std::int64_t row = 0; row < gridHeight; row++) {
        for(std::int64_t column = 0; column < gridWidth; column++) {
            const std::array<float, 2> center{static_cast<float>(column) * static_cast<float>(stride), static_cast<float>(row) * static_cast<float>(stride)};
            for(std::int64_t duplicate = 0; duplicate < duplicates; duplicate++) {
                anchorCenters.push_back(center);
            }
        }
    }
    return anchorCenters;
}

std::vector<std::array<float, 4>> distance2Bbox(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance) {
    DAI_CHECK_V(distance.size() == points.size() * BBOX_VALUES_PER_ANCHOR,
                "Expected {} bounding box distance values for {} anchor centers, got {}.",
                points.size() * BBOX_VALUES_PER_ANCHOR,
                points.size(),
                distance.size());

    std::vector<std::array<float, 4>> bboxes(points.size());
    for(std::size_t i = 0; i < points.size(); i++) {
        const float* row = distance.data() + i * BBOX_VALUES_PER_ANCHOR;
        bboxes[i][0] = points[i][0] - row[0];
        bboxes[i][1] = points[i][1] - row[1];
        bboxes[i][2] = points[i][0] + row[2];
        bboxes[i][3] = points[i][1] + row[3];
    }
    return bboxes;
}

std::vector<float> distance2Kps(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance, std::size_t valuesPerPoint) {
    DAI_CHECK_V(valuesPerPoint > 0 && valuesPerPoint % 2 == 0, "Keypoint distance values per point must be a positive multiple of 2, got {}.", valuesPerPoint);
    DAI_CHECK_V(distance.size() == points.size() * valuesPerPoint,
                "Expected {} keypoint distance values for {} anchor centers, got {}.",
                points.size() * valuesPerPoint,
                points.size(),
                distance.size());

    std::vector<float> preds(distance.size());
    for(std::size_t n = 0; n < points.size(); n++) {
        const float* row = distance.data() + n * valuesPerPoint;
        float* out = preds.data() + n * valuesPerPoint;
        for(std::size_t i = 0; i < valuesPerPoint; i += 2) {
            // px = points[:, i % 2] + distance[:, i]; py = points[:, i % 2 + 1] + distance[:, i + 1].
            // i is always even, so i % 2 selects the anchor center x and i % 2 + 1 its y.
            out[i] = points[n][i % 2] + row[i];
            out[i + 1] = points[n][i % 2 + 1] + row[i + 1];
        }
    }
    return preds;
}

std::vector<std::int64_t> nms(const std::vector<std::array<float, 5>>& dets, float nmsThreshold) {
    const std::size_t num = dets.size();
    std::vector<float> scores(num);
    // areas = (x2 - x1 + 1) * (y2 - y1 + 1), single precision.
    std::vector<float> areas(num);
    for(std::size_t i = 0; i < num; i++) {
        scores[i] = dets[i][4];
        areas[i] = (dets[i][2] - dets[i][0] + 1.0f) * (dets[i][3] - dets[i][1] + 1.0f);
    }

    // order = scores.argsort()[::-1]: ascending numpy argsort reversed.
    std::vector<std::int64_t> order = MLSDUtils::argsortAscending(scores);
    std::reverse(order.begin(), order.end());

    std::vector<std::int64_t> keep;
    while(!order.empty()) {
        const std::int64_t i = order.front();
        keep.push_back(i);

        std::vector<std::int64_t> remaining;
        remaining.reserve(order.size() - 1);
        for(std::size_t k = 1; k < order.size(); k++) {
            const std::int64_t j = order[k];
            const float xx1 = npMaximum(dets[i][0], dets[j][0]);
            const float yy1 = npMaximum(dets[i][1], dets[j][1]);
            const float xx2 = npMinimum(dets[i][2], dets[j][2]);
            const float yy2 = npMinimum(dets[i][3], dets[j][3]);

            const float w = npMaximum(0.0f, xx2 - xx1 + 1.0f);
            const float h = npMaximum(0.0f, yy2 - yy1 + 1.0f);
            const float inter = w * h;
            const float ovr = inter / (areas[i] + areas[j] - inter);

            // inds = np.where(ovr <= thresh)[0]: kept when the overlap is at most the
            // threshold (inclusive); NaN overlaps compare false and are suppressed.
            if(ovr <= nmsThreshold) {
                remaining.push_back(j);
            }
        }
        order = std::move(remaining);
    }
    return keep;
}

ScrfdDetections computeScrfdDetections(const std::vector<std::vector<float>>& bboxesPerStride,
                                       const std::vector<std::vector<float>>& scoresPerStride,
                                       const std::vector<std::vector<float>>& kpsPerStride,
                                       const std::vector<std::int64_t>& featStrideFpn,
                                       std::uint32_t inputWidth,
                                       std::uint32_t inputHeight,
                                       float scoreThreshold,
                                       float nmsThreshold,
                                       int maxDetections,
                                       const std::vector<std::vector<std::array<float, 2>>>& anchors) {
    DAI_CHECK(!featStrideFpn.empty(), "SCRFD decoding requires at least one feature stride.");
    DAI_CHECK(maxDetections > 0, "Maximum detections must be positive.");
    DAI_CHECK_V(bboxesPerStride.size() == featStrideFpn.size() && scoresPerStride.size() == featStrideFpn.size() && kpsPerStride.size() == featStrideFpn.size()
                    && anchors.size() == featStrideFpn.size(),
                "Expected {} per-stride score, bbox, kps and anchor vectors, got {}, {}, {} and {}.",
                featStrideFpn.size(),
                scoresPerStride.size(),
                bboxesPerStride.size(),
                kpsPerStride.size(),
                anchors.size());

    std::vector<float> allScores;
    std::vector<std::array<float, 4>> allBboxes;
    std::vector<std::array<float, KPS_VALUES_PER_ANCHOR>> allKps;

    for(std::size_t idx = 0; idx < featStrideFpn.size(); idx++) {
        const std::int64_t stride = featStrideFpn[idx];
        const auto& scores = scoresPerStride[idx];
        const auto& anchorCenters = anchors[idx];
        const std::size_t numPositions = scores.size();
        DAI_CHECK_V(bboxesPerStride[idx].size() == numPositions * BBOX_VALUES_PER_ANCHOR,
                    "SCRFD stride {}: expected {} bbox values for {} scores, got {}.",
                    stride,
                    numPositions * BBOX_VALUES_PER_ANCHOR,
                    numPositions,
                    bboxesPerStride[idx].size());
        DAI_CHECK_V(kpsPerStride[idx].size() == numPositions * KPS_VALUES_PER_ANCHOR,
                    "SCRFD stride {}: expected {} kps values for {} scores, got {}.",
                    stride,
                    numPositions * KPS_VALUES_PER_ANCHOR,
                    numPositions,
                    kpsPerStride[idx].size());
        // The source relies on numpy broadcasting, which requires the tensor positions to
        // match the anchor grid derived from the input size, stride and number of anchors.
        DAI_CHECK_V(anchorCenters.size() == numPositions,
                    "SCRFD stride {}: got {} scores but {} anchor centers; the tensor size must match the anchor grid derived from the input size ({}, {}), "
                    "the stride and the number of anchors.",
                    stride,
                    numPositions,
                    anchorCenters.size(),
                    inputWidth,
                    inputHeight);

        // bbox_preds = bboxes * stride; kps_preds = kps * stride (single precision).
        std::vector<float> bboxPreds(bboxesPerStride[idx].size());
        for(std::size_t i = 0; i < bboxPreds.size(); i++) {
            bboxPreds[i] = bboxesPerStride[idx][i] * static_cast<float>(stride);
        }
        std::vector<float> kpsPreds(kpsPerStride[idx].size());
        for(std::size_t i = 0; i < kpsPreds.size(); i++) {
            kpsPreds[i] = kpsPerStride[idx][i] * static_cast<float>(stride);
        }

        const auto decodedBboxes = distance2Bbox(anchorCenters, bboxPreds);
        const auto decodedKps = distance2Kps(anchorCenters, kpsPreds, KPS_VALUES_PER_ANCHOR);

        // pos_inds = np.where(scores >= score_threshold)[0]: the threshold is inclusive.
        for(std::size_t n = 0; n < numPositions; n++) {
            if(scores[n] >= scoreThreshold) {
                allScores.push_back(scores[n]);
                allBboxes.push_back(decodedBboxes[n]);
                std::array<float, KPS_VALUES_PER_ANCHOR> kpsRow{};
                std::copy_n(decodedKps.data() + n * KPS_VALUES_PER_ANCHOR, KPS_VALUES_PER_ANCHOR, kpsRow.begin());
                allKps.push_back(kpsRow);
            }
        }
    }

    // order = scores.ravel().argsort()[::-1]: ascending numpy argsort reversed.
    std::vector<std::int64_t> order = MLSDUtils::argsortAscending(allScores);
    std::reverse(order.begin(), order.end());

    // pre_det = np.hstack((bboxes, scores))[order]; kpss = kpss[order].
    std::vector<std::array<float, 5>> preDet(order.size());
    std::vector<std::array<float, KPS_VALUES_PER_ANCHOR>> orderedKps(order.size());
    for(std::size_t k = 0; k < order.size(); k++) {
        const std::int64_t src = order[k];
        preDet[k] = {allBboxes[src][0], allBboxes[src][1], allBboxes[src][2], allBboxes[src][3], allScores[src]};
        orderedKps[k] = allKps[src];
    }

    const std::vector<std::int64_t> keep = nms(preDet, nmsThreshold);
    const std::size_t numDetections = std::min(keep.size(), static_cast<std::size_t>(maxDetections));

    // Normalize in double precision: the source divides float32 values by int64 arrays, which
    // numpy promotes to float64. x coordinates are normalized by the input width and y
    // coordinates by the input height.
    const double width = static_cast<double>(inputWidth);
    const double height = static_cast<double>(inputHeight);

    ScrfdDetections detections;
    detections.bboxes.reserve(numDetections);
    detections.scores.reserve(numDetections);
    detections.keypoints.reserve(numDetections);
    for(std::size_t detectionIndex = 0; detectionIndex < numDetections; detectionIndex++) {
        const std::int64_t keptIdx = keep[detectionIndex];
        const auto& row = preDet[keptIdx];
        // bboxes = np.clip(det[:, :4] / [width, height, width, height], 0, 1).
        const double x1 = clip01(static_cast<double>(row[0]) / width);
        const double y1 = clip01(static_cast<double>(row[1]) / height);
        const double x2 = clip01(static_cast<double>(row[2]) / width);
        const double y2 = clip01(static_cast<double>(row[3]) / height);
        // xyxy_to_xywh validation on the clipped coordinates.
        DAI_CHECK(x1 <= x2 && y1 <= y2, "Bounding box coordinates must be in the format [x_min, y_min, x_max, y_max].");
        // xyxy -> center xywh, clipped to [0, 1] again; cast to single precision like the
        // source message creator's float() casts.
        detections.bboxes.push_back({static_cast<float>(clip01((x1 + x2) / 2.0)),
                                     static_cast<float>(clip01((y1 + y2) / 2.0)),
                                     static_cast<float>(clip01(x2 - x1)),
                                     static_cast<float>(clip01(y2 - y1))});
        detections.scores.push_back(row[4]);

        // keypoints = np.clip(kpss / np.tile([width, height], (5, 1)), 0, 1).
        const auto& kpsRow = orderedKps[keptIdx];
        std::vector<std::array<float, 2>> keypoints(NUM_KEYPOINTS);
        for(std::size_t kp = 0; kp < NUM_KEYPOINTS; kp++) {
            keypoints[kp][0] = static_cast<float>(clip01(static_cast<double>(kpsRow[2 * kp]) / width));
            keypoints[kp][1] = static_cast<float>(clip01(static_cast<double>(kpsRow[2 * kp + 1]) / height));
        }
        detections.keypoints.push_back(std::move(keypoints));
    }
    return detections;
}

}  // namespace SCRFDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
