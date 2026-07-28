// This file contains utility functions for decoding the output of the FastSAM instance
// segmentation model, ported from the depthai-nodes parser utilities (utils/fastsam.py,
// utils/yolo.py, utils/nms.py and utils/masks_utils.py).
//
// Several functions are based on code from the Ultralytics FastSAM implementation; the
// original per-function source attributions of the depthai-nodes utilities are retained
// verbatim in the function documentation in FastSAMUtils.hpp.

#include "beta/utilities/FastSAM/FastSAMUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <vector>

#include "beta/utilities/Detection/MaskUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace FastSAMUtils {

namespace {

// The source non_max_suppression() defaults; the FastSAM parser never overrides them.
constexpr std::size_t kMaxNms = 3000;      // decode_yolo_output max_nms
constexpr std::size_t kMaxDet = 300;       // non_max_suppression max_det
constexpr double kMaxWh = 7680.0;          // non_max_suppression max_wh class offset
constexpr double kFullBoxIouThres = 0.9;   // decode_fastsam_output bbox_iou threshold
constexpr std::int64_t kBorderThres = 20;  // adjust_bboxes_to_image_border threshold

/**
 * A per-grid-cell candidate produced by the anchorless YOLO parse, before non-maximum
 * suppression. Matches one row of the source parse_yolo_output() segmentation output
 * [cx, cy, w, h, obj, cls..., head, anchor, x, y] with the class scores already reduced to
 * their maximum and its index (the source single-label path computes the same reduction).
 */
struct Candidate {
    double cx = 0.0, cy = 0.0, w = 0.0, h = 0.0;
    double obj = 0.0;
    double clsConf = 0.0;
    std::int64_t clsIdx = 0;
    std::int64_t head = 0, anchor = 0, gx = 0, gy = 0;
};

/**
 * Parse a single anchorless YOLO head output in segmentation mode, mirroring the source
 * parse_yolo_output() with anchors=None, kpts=None, det_mode=False and the DEFAULT subtype.
 */
std::vector<Candidate> parseYoloHead(const ClassificationUtils::ShapedTensorData& tensor, std::int64_t stride, std::int32_t numClasses, std::int64_t headId) {
    DAI_CHECK_V(tensor.dims.size() == 4, "FastSAM YOLO output at index {} must be a 4D NCHW tensor, got {} dimensions.", headId, tensor.dims.size());
    DAI_CHECK_V(tensor.dims[0] == 1, "FastSAM YOLO output at index {} must have batch size 1, got {}.", headId, tensor.dims[0]);
    const std::size_t channels = tensor.dims[1];
    const std::size_t ny = tensor.dims[2];
    const std::size_t nx = tensor.dims[3];
    DAI_CHECK_V(channels == static_cast<std::size_t>(numClasses) + 5,
                "FastSAM YOLO output at index {} has {} channels, expected numClasses + 5 = {}.",
                headId,
                channels,
                numClasses + 5);
    DAI_CHECK_V(tensor.values.size() == channels * ny * nx,
                "FastSAM YOLO output at index {} holds {} values, expected {} for shape (1, {}, {}, {}).",
                headId,
                tensor.values.size(),
                channels * ny * nx,
                channels,
                ny,
                nx);

    const auto at = [&](std::size_t channel, std::size_t y, std::size_t x) -> float { return tensor.values[(channel * ny + y) * nx + x]; };

    std::vector<Candidate> candidates;
    candidates.reserve(ny * nx);
    // Row order matches the source reshape(bs, na * ny * nx, -1): y-major, then x (na = 1).
    for(std::size_t y = 0; y < ny; ++y) {
        for(std::size_t x = 0; x < nx; ++x) {
            Candidate c;
            // Anchorless box decode: x1y1 = grid - out[0:2] + 0.5, x2y2 = grid + out[2:4] + 0.5,
            // then center/size scaled by the stride. The results are stored into the float32
            // array in the source (out[..., 0:2] = c_xy * stride), so they round through
            // float32 before the float64 concatenation.
            const double x1 = static_cast<double>(x) - static_cast<double>(at(0, y, x)) + 0.5;
            const double y1 = static_cast<double>(y) - static_cast<double>(at(1, y, x)) + 0.5;
            const double x2 = static_cast<double>(x) + static_cast<double>(at(2, y, x)) + 0.5;
            const double y2 = static_cast<double>(y) + static_cast<double>(at(3, y, x)) + 0.5;
            c.cx = static_cast<double>(static_cast<float>((x1 + x2) / 2.0 * static_cast<double>(stride)));
            c.cy = static_cast<double>(static_cast<float>((y1 + y2) / 2.0 * static_cast<double>(stride)));
            c.w = static_cast<double>(static_cast<float>((x2 - x1) * static_cast<double>(stride)));
            c.h = static_cast<double>(static_cast<float>((y2 - y1) * static_cast<double>(stride)));
            c.obj = static_cast<double>(at(4, y, x));

            // Single-label reduction (multi_label is always False here): the class with the
            // maximum score wins, following numpy argmax semantics (first occurrence on ties,
            // the first NaN wins when present).
            double best = static_cast<double>(at(5, y, x));
            std::int64_t bestIdx = 0;
            for(std::int32_t k = 1; k < numClasses; ++k) {
                const double v = static_cast<double>(at(5 + static_cast<std::size_t>(k), y, x));
                if(v > best || (std::isnan(v) && !std::isnan(best))) {
                    best = v;
                    bestIdx = k;
                }
            }
            c.clsConf = best;
            c.clsIdx = bestIdx;

            c.head = headId;
            c.anchor = 0;
            c.gx = static_cast<std::int64_t>(x);
            c.gy = static_cast<std::int64_t>(y);
            candidates.push_back(c);
        }
    }
    return candidates;
}

/// numpy argmax over doubles: first occurrence of the maximum, the first NaN wins when present.
std::size_t argmaxNumpy(const std::vector<double>& values) {
    std::size_t bestIdx = 0;
    double best = values[0];
    for(std::size_t i = 1; i < values.size(); ++i) {
        const double v = values[i];
        if(v > best || (std::isnan(v) && !std::isnan(best))) {
            best = v;
            bestIdx = i;
        }
    }
    return bestIdx;
}

}  // namespace

std::vector<std::size_t> nms(const std::vector<std::array<float, 4>>& boxes, const std::vector<float>& scores, float nmsThreshold) {
    DAI_CHECK_V(boxes.size() == scores.size(), "NMS requires one score per box, got {} boxes and {} scores.", boxes.size(), scores.size());
    const std::size_t n = boxes.size();

    // areas = (x2 - x1 + 1) * (y2 - y1 + 1), computed in float32 like the source, which casts
    // the stacked (boxes, scores) array to float32 before calling nms().
    std::vector<float> areas(n);
    for(std::size_t i = 0; i < n; ++i) {
        areas[i] = (boxes[i][2] - boxes[i][0] + 1.0f) * (boxes[i][3] - boxes[i][1] + 1.0f);
    }

    // order = scores.argsort()[::-1]: ascending stable argsort reversed, so equal scores are
    // visited with the later index first.
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&scores](std::size_t a, std::size_t b) { return scores[a] < scores[b]; });
    std::reverse(order.begin(), order.end());

    std::vector<std::size_t> keep;
    while(!order.empty()) {
        const std::size_t i = order.front();
        keep.push_back(i);
        std::vector<std::size_t> remaining;
        remaining.reserve(order.size());
        for(std::size_t k = 1; k < order.size(); ++k) {
            const std::size_t j = order[k];
            const float xx1 = std::max(boxes[i][0], boxes[j][0]);
            const float yy1 = std::max(boxes[i][1], boxes[j][1]);
            const float xx2 = std::min(boxes[i][2], boxes[j][2]);
            const float yy2 = std::min(boxes[i][3], boxes[j][3]);
            const float w = std::max(0.0f, xx2 - xx1 + 1.0f);
            const float h = std::max(0.0f, yy2 - yy1 + 1.0f);
            const float inter = w * h;
            const float ovr = inter / (areas[i] + areas[j] - inter);
            // Keep boxes with ovr <= thresh; a NaN overlap compares false and is suppressed,
            // matching the numpy comparison.
            if(ovr <= nmsThreshold) {
                remaining.push_back(j);
            }
        }
        order = std::move(remaining);
    }
    return keep;
}

void adjustBboxesToImageBorder(std::vector<DetectionRow>& rows, std::int64_t imgHeight, std::int64_t imgWidth, std::int64_t threshold) {
    for(auto& row : rows) {
        if(row[0] < static_cast<double>(threshold)) row[0] = 0.0;                                         // x1
        if(row[1] < static_cast<double>(threshold)) row[1] = 0.0;                                         // y1
        if(row[2] > static_cast<double>(imgWidth - threshold)) row[2] = static_cast<double>(imgWidth);    // x2
        if(row[3] > static_cast<double>(imgHeight - threshold)) row[3] = static_cast<double>(imgHeight);  // y2
    }
}

std::vector<std::size_t> bboxIou(
    const std::array<double, 4>& box1, std::vector<DetectionRow>& rows, std::int64_t imgHeight, std::int64_t imgWidth, double iouThres) {
    // The source bbox_iou() border-snaps the boxes in place before computing the IoU; the
    // mutation is visible to the caller and drives the later mask cropping.
    adjustBboxesToImageBorder(rows, imgHeight, imgWidth, kBorderThres);

    const double box1Area = (box1[2] - box1[0]) * (box1[3] - box1[1]);
    std::vector<std::size_t> indices;
    for(std::size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i];
        const double x1 = std::max(box1[0], row[0]);
        const double y1 = std::max(box1[1], row[1]);
        const double x2 = std::min(box1[2], row[2]);
        const double y2 = std::min(box1[3], row[3]);
        const double intersection = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
        const double box2Area = (row[2] - row[0]) * (row[3] - row[1]);
        const double unionArea = box1Area + box2Area - intersection;
        const double iou = intersection / unionArea;
        if(iou > iouThres) {
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<DetectionRow> decodeFastsamOutput(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,
                                              std::int64_t imgHeight,
                                              std::int64_t imgWidth,
                                              float confThres,
                                              float iouThres,
                                              std::int32_t numClasses) {
    // The source compute_fastsam_mask() always decodes with strides [8, 16, 32] and no
    // anchors; decode_yolo_output() validates the stride/output count match.
    const std::vector<std::int64_t> strides = {8, 16, 32};
    DAI_CHECK_V(strides.size() == yoloOutputs.size(),
                "Number of `strides` must match number of YOLO outputs. Got {} strides for {} outputs.",
                strides.size(),
                yoloOutputs.size());
    DAI_CHECK_V(confThres >= 0.0f && confThres <= 1.0f, "Invalid Confidence threshold {}, valid values are between 0.0 and 1.0.", confThres);
    DAI_CHECK_V(iouThres >= 0.0f && iouThres <= 1.0f, "Invalid IoU {}, valid values are between 0.0 and 1.0.", iouThres);

    // 1. Parse all head outputs and keep only candidates with objectness > confThres (the
    // source early filter; the non_max_suppression objectness re-filter selects the same set).
    std::vector<Candidate> candidates;
    for(std::size_t i = 0; i < yoloOutputs.size(); ++i) {
        const auto headCandidates = parseYoloHead(yoloOutputs[i], strides[i], numClasses, static_cast<std::int64_t>(i));
        for(const auto& c : headCandidates) {
            if(c.obj > static_cast<double>(confThres)) {
                candidates.push_back(c);
            }
        }
    }
    if(candidates.empty()) {
        return {};
    }

    // 2. When more than max_nms candidates with non-identical objectness remain, keep the
    // max_nms highest-objectness candidates (descending sort; equal values keep the reversed
    // ascending-argsort order, i.e. the later index first).
    if(candidates.size() > kMaxNms) {
        const bool allEqual = std::all_of(candidates.begin(), candidates.end(), [&candidates](const Candidate& c) { return c.obj == candidates.front().obj; });
        if(!allEqual) {
            std::vector<std::size_t> order(candidates.size());
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(), [&candidates](std::size_t a, std::size_t b) { return candidates[a].obj < candidates[b].obj; });
            std::reverse(order.begin(), order.end());
            order.resize(kMaxNms);
            std::vector<Candidate> truncated;
            truncated.reserve(kMaxNms);
            for(const std::size_t idx : order) {
                truncated.push_back(candidates[idx]);
            }
            candidates = std::move(truncated);
        }
    }

    // 3. Non-maximum suppression (source non_max_suppression(), single-label path): convert
    // boxes to corner form and re-filter with the class confidence.
    struct NmsRow {
        double x1, y1, x2, y2;
        double conf;
        std::int64_t clsIdx;
        std::int64_t head, anchor, gx, gy;
    };
    std::vector<NmsRow> nmsRows;
    nmsRows.reserve(candidates.size());
    for(const auto& c : candidates) {
        if(c.clsConf > static_cast<double>(confThres)) {
            NmsRow row;
            row.x1 = c.cx - c.w / 2.0;
            row.y1 = c.cy - c.h / 2.0;
            row.x2 = c.cx + c.w / 2.0;
            row.y2 = c.cy + c.h / 2.0;
            row.conf = c.clsConf;
            row.clsIdx = c.clsIdx;
            row.head = c.head;
            row.anchor = c.anchor;
            row.gx = c.gx;
            row.gy = c.gy;
            nmsRows.push_back(row);
        }
    }
    if(nmsRows.empty()) {
        return {};
    }

    // Source quirk kept for parity: when more than max_nms boxes remain, the source sorts by
    // ascending confidence and keeps the first max_nms rows (x[x[:, 4].argsort()[:max_nms]]),
    // i.e. the lowest-confidence rows, and the boxes enter NMS in that ascending order.
    if(nmsRows.size() > kMaxNms) {
        std::vector<std::size_t> order(nmsRows.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&nmsRows](std::size_t a, std::size_t b) { return nmsRows[a].conf < nmsRows[b].conf; });
        order.resize(kMaxNms);
        std::vector<NmsRow> truncated;
        truncated.reserve(kMaxNms);
        for(const std::size_t idx : order) {
            truncated.push_back(nmsRows[idx]);
        }
        nmsRows = std::move(truncated);
    }

    // Batched NMS: boxes offset by class * max_wh, then the float32 nms() of utils/nms.py.
    std::vector<std::array<float, 4>> nmsBoxes(nmsRows.size());
    std::vector<float> nmsScores(nmsRows.size());
    for(std::size_t i = 0; i < nmsRows.size(); ++i) {
        const double offset = static_cast<double>(nmsRows[i].clsIdx) * kMaxWh;
        nmsBoxes[i] = {static_cast<float>(nmsRows[i].x1 + offset),
                       static_cast<float>(nmsRows[i].y1 + offset),
                       static_cast<float>(nmsRows[i].x2 + offset),
                       static_cast<float>(nmsRows[i].y2 + offset)};
        nmsScores[i] = static_cast<float>(nmsRows[i].conf);
    }
    std::vector<std::size_t> keep = nms(nmsBoxes, nmsScores, iouThres);
    if(keep.size() > kMaxDet) {
        keep.resize(kMaxDet);
    }

    std::vector<DetectionRow> rows;
    rows.reserve(keep.size());
    for(const std::size_t idx : keep) {
        const auto& r = nmsRows[idx];
        rows.push_back(DetectionRow{r.x1,
                                    r.y1,
                                    r.x2,
                                    r.y2,
                                    r.conf,
                                    static_cast<double>(r.clsIdx),
                                    static_cast<double>(r.head),
                                    static_cast<double>(r.anchor),
                                    static_cast<double>(r.gx),
                                    static_cast<double>(r.gy)});
    }

    // 4. Full-box augmentation (source decode_fastsam_output()): border-snap all boxes in
    // place through bboxIou() and replace the highest-confidence box overlapping the full
    // image (IoU > 0.9) with the full-image box, keeping its confidence and grid-cell columns
    // and setting its class to 0.
    const std::array<double, 4> fullBox = {0.0, 0.0, static_cast<double>(imgWidth), static_cast<double>(imgHeight)};
    const std::vector<std::size_t> criticalIndices = bboxIou(fullBox, rows, imgHeight, imgWidth, kFullBoxIouThres);
    if(!criticalIndices.empty()) {
        std::vector<double> confs(criticalIndices.size());
        for(std::size_t i = 0; i < criticalIndices.size(); ++i) {
            confs[i] = rows[criticalIndices[i]][4];
        }
        const std::size_t idx = criticalIndices[argmaxNumpy(confs)];
        rows[idx][0] = 0.0;
        rows[idx][1] = 0.0;
        rows[idx][2] = static_cast<double>(imgWidth);
        rows[idx][3] = static_cast<double>(imgHeight);
        rows[idx][5] = 0.0;
        // Columns 4 (confidence) and 6..9 (head, anchor, x, y) keep the replaced row's values.
    }

    return rows;
}

std::vector<float> buildMaskCoeffs(const std::vector<DetectionRow>& rows,
                                   const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                   std::size_t protosLen) {
    // The source leaves rows whose head index matches no mask output uninitialized
    // (np.empty); this port zero-initializes them instead.
    std::vector<float> coeffs(rows.size() * protosLen, 0.0f);
    for(std::size_t i = 0; i < rows.size(); ++i) {
        const auto headIdx = static_cast<std::int64_t>(rows[i][6]);
        const auto anchorIdx = static_cast<std::int64_t>(rows[i][7]);
        const auto x = static_cast<std::int64_t>(rows[i][8]);
        const auto y = static_cast<std::int64_t>(rows[i][9]);
        if(headIdx < 0 || headIdx >= static_cast<std::int64_t>(maskOutputs.size())) {
            continue;
        }
        const auto& mask = maskOutputs[static_cast<std::size_t>(headIdx)];
        DAI_CHECK_V(mask.dims.size() == 4, "FastSAM mask output at index {} must be a 4D NCHW tensor, got {} dimensions.", headIdx, mask.dims.size());
        const std::size_t channels = mask.dims[1];
        const std::size_t ny = mask.dims[2];
        const std::size_t nx = mask.dims[3];
        DAI_CHECK_V(anchorIdx >= 0 && (static_cast<std::size_t>(anchorIdx) + 1) * protosLen <= channels,
                    "FastSAM mask output at index {} has {} channels, expected at least {} for anchor {} with {} prototypes.",
                    headIdx,
                    channels,
                    (anchorIdx + 1) * static_cast<std::int64_t>(protosLen),
                    anchorIdx,
                    protosLen);
        DAI_CHECK_V(x >= 0 && static_cast<std::size_t>(x) < nx && y >= 0 && static_cast<std::size_t>(y) < ny,
                    "FastSAM mask output at index {} with grid ({}, {}) cannot address grid cell ({}, {}); the mask output grid must match its YOLO "
                    "output grid.",
                    headIdx,
                    ny,
                    nx,
                    y,
                    x);
        for(std::size_t k = 0; k < protosLen; ++k) {
            const std::size_t channel = static_cast<std::size_t>(anchorIdx) * protosLen + k;
            coeffs[i * protosLen + k] = mask.values[(channel * ny + static_cast<std::size_t>(y)) * nx + static_cast<std::size_t>(x)];
        }
    }
    return coeffs;
}

std::vector<std::uint8_t> processMasks(const std::vector<DetectionRow>& rows,
                                       const std::vector<float>& maskCoeffs,
                                       const std::vector<float>& protos,
                                       std::size_t protosLen,
                                       std::size_t protoHeight,
                                       std::size_t protoWidth,
                                       std::size_t outHeight,
                                       std::size_t outWidth,
                                       float maskConf) {
    DAI_CHECK_V(protos.size() == protosLen * protoHeight * protoWidth,
                "FastSAM protos tensor holds {} values, expected {} for shape ({}, {}, {}).",
                protos.size(),
                protosLen * protoHeight * protoWidth,
                protosLen,
                protoHeight,
                protoWidth);
    DAI_CHECK_V(maskCoeffs.size() == rows.size() * protosLen,
                "FastSAM mask coefficients hold {} values, expected {} for {} detections with {} prototypes.",
                maskCoeffs.size(),
                rows.size() * protosLen,
                rows.size(),
                protosLen);

    std::vector<std::uint8_t> masks(rows.size() * outHeight * outWidth);
    std::vector<float> maskSmall(protoHeight * protoWidth);
    for(std::size_t i = 0; i < rows.size(); ++i) {
        // mask_small = sigmoid(sum(protos * coeffs, axis=0)): sequential float32 accumulation
        // over the prototypes, matching numpy's axis-0 sum of a float32 array.
        std::fill(maskSmall.begin(), maskSmall.end(), 0.0f);
        for(std::size_t c = 0; c < protosLen; ++c) {
            const float coeff = maskCoeffs[i * protosLen + c];
            const float* plane = protos.data() + c * protoHeight * protoWidth;
            for(std::size_t p = 0; p < protoHeight * protoWidth; ++p) {
                maskSmall[p] += plane[p] * coeff;
            }
        }
        for(auto& v : maskSmall) {
            v = MaskUtils::sigmoid(v);
        }

        const std::vector<float> resized = MaskUtils::resizeNearest(maskSmall, protoHeight, protoWidth, outHeight, outWidth);

        // Bounding box truncated to int and clamped to [0, outWidth/outHeight]; the mask is
        // zeroed outside the box before the strict greater-than threshold.
        const auto clampBox = [](double v, std::int64_t hi) { return std::min(std::max(static_cast<std::int64_t>(v), std::int64_t{0}), hi); };
        const std::int64_t x1 = clampBox(rows[i][0], static_cast<std::int64_t>(outWidth));
        const std::int64_t y1 = clampBox(rows[i][1], static_cast<std::int64_t>(outHeight));
        const std::int64_t x2 = clampBox(rows[i][2], static_cast<std::int64_t>(outWidth));
        const std::int64_t y2 = clampBox(rows[i][3], static_cast<std::int64_t>(outHeight));

        std::uint8_t* out = masks.data() + i * outHeight * outWidth;
        for(std::size_t row = 0; row < outHeight; ++row) {
            const bool rowInside = static_cast<std::int64_t>(row) >= y1 && static_cast<std::int64_t>(row) < y2;
            for(std::size_t col = 0; col < outWidth; ++col) {
                const bool inside = rowInside && static_cast<std::int64_t>(col) >= x1 && static_cast<std::int64_t>(col) < x2;
                out[row * outWidth + col] = (inside && resized[row * outWidth + col] > maskConf) ? 1 : 0;
            }
        }
    }
    return masks;
}

std::vector<std::uint8_t> boxPrompt(
    const std::vector<std::uint8_t>& masks, std::size_t maskCount, std::size_t height, std::size_t width, const std::array<std::int32_t, 4>& bbox) {
    DAI_CHECK(maskCount > 0, "FastSAM box prompt requires at least one mask.");
    DAI_CHECK(bbox[2] != 0 && bbox[3] != 0, "FastSAM box prompt requires a bounding box with non-zero x2 and y2 coordinates.");

    // The source rescales the box when the mask shape differs from the original shape; in this
    // parser both are the model input size, so only the clamp applies.
    const std::int64_t x1 = std::max(static_cast<std::int64_t>(bbox[0]), std::int64_t{0});
    const std::int64_t y1 = std::max(static_cast<std::int64_t>(bbox[1]), std::int64_t{0});
    const std::int64_t x2 = std::min(static_cast<std::int64_t>(bbox[2]), static_cast<std::int64_t>(width));
    const std::int64_t y2 = std::min(static_cast<std::int64_t>(bbox[3]), static_cast<std::int64_t>(height));

    const std::int64_t bboxArea = (y2 - y1) * (x2 - x1);

    std::vector<double> ious(maskCount);
    for(std::size_t i = 0; i < maskCount; ++i) {
        const std::uint8_t* mask = masks.data() + i * height * width;
        std::int64_t masksArea = 0;
        for(std::int64_t row = y1; row < y2; ++row) {
            for(std::int64_t col = x1; col < x2; ++col) {
                masksArea += mask[static_cast<std::size_t>(row) * width + static_cast<std::size_t>(col)];
            }
        }
        std::int64_t origMasksArea = 0;
        for(std::size_t p = 0; p < height * width; ++p) {
            origMasksArea += mask[p];
        }
        const std::int64_t unionArea = bboxArea + origMasksArea - masksArea;
        // Integer / integer promotes to float64 in the source; 0/0 yields NaN, which wins the
        // argmax like numpy.
        ious[i] = static_cast<double>(masksArea) / static_cast<double>(unionArea);
    }
    const std::size_t maxIouIndex = argmaxNumpy(ious);

    std::vector<std::uint8_t> result(height * width);
    std::copy_n(masks.data() + maxIouIndex * height * width, height * width, result.data());
    return result;
}

std::vector<std::uint8_t> pointPrompt(const std::vector<std::uint8_t>& masks,
                                      std::size_t maskCount,
                                      std::size_t height,
                                      std::size_t width,
                                      const std::pair<std::int32_t, std::int32_t>& point,
                                      std::int32_t pointLabel) {
    DAI_CHECK(maskCount > 0, "FastSAM point prompt requires at least one mask.");

    // Negative coordinates wrap around like numpy indexing; out-of-range coordinates raise
    // like the source IndexError.
    std::int64_t px = point.first;
    std::int64_t py = point.second;
    if(px < 0) px += static_cast<std::int64_t>(width);
    if(py < 0) py += static_cast<std::int64_t>(height);
    DAI_CHECK_V(px >= 0 && px < static_cast<std::int64_t>(width) && py >= 0 && py < static_cast<std::int64_t>(height),
                "FastSAM point prompt point ({}, {}) is out of bounds for a mask of shape ({}, {}).",
                point.first,
                point.second,
                height,
                width);

    std::vector<std::int32_t> accumulator(height * width, 0);
    for(std::size_t i = 0; i < maskCount; ++i) {
        const std::uint8_t* mask = masks.data() + i * height * width;
        if(mask[static_cast<std::size_t>(py) * width + static_cast<std::size_t>(px)] != 1) {
            continue;
        }
        if(pointLabel == 1) {
            for(std::size_t p = 0; p < height * width; ++p) {
                accumulator[p] += mask[p];
            }
        } else if(pointLabel == 0) {
            for(std::size_t p = 0; p < height * width; ++p) {
                accumulator[p] -= mask[p];
            }
        }
    }

    std::vector<std::uint8_t> result(height * width);
    for(std::size_t p = 0; p < height * width; ++p) {
        result[p] = accumulator[p] >= 1 ? 1 : 0;
    }
    return result;
}

std::vector<std::uint8_t> mergeMasks(const std::vector<std::uint8_t>& masks, std::size_t maskCount, std::size_t height, std::size_t width) {
    DAI_CHECK_V(masks.size() == maskCount * height * width,
                "FastSAM masks hold {} values, expected {} for shape ({}, {}, {}).",
                masks.size(),
                maskCount * height * width,
                maskCount,
                height,
                width);
    // Instance index 256 does not fit uint8 (the source numpy assignment raises an
    // OverflowError); 255 is the background value.
    DAI_CHECK_V(maskCount <= 256, "FastSAM cannot merge {} masks; at most 256 instance indices fit into a uint8 segmentation mask.", maskCount);

    std::vector<std::uint8_t> merged(height * width, 255);
    for(std::size_t i = 0; i < maskCount; ++i) {
        const std::uint8_t* mask = masks.data() + i * height * width;
        for(std::size_t p = 0; p < height * width; ++p) {
            if(mask[p] > 0) {
                merged[p] = static_cast<std::uint8_t>(i);
            }
        }
    }
    return merged;
}

FastsamResult computeFastsamMask(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,
                                 const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                 const ClassificationUtils::ShapedTensorData& protos,
                                 std::size_t protosLen,
                                 float confThreshold,
                                 std::int32_t numClasses,
                                 float iouThreshold,
                                 float maskConf,
                                 const PromptConfig& promptConfig) {
    DAI_CHECK(!yoloOutputs.empty(), "FastSAMParser requires at least one YOLO output tensor.");
    DAI_CHECK_V(
        yoloOutputs.front().dims.size() == 4, "FastSAM YOLO output at index 0 must be a 4D NCHW tensor, got {} dimensions.", yoloOutputs.front().dims.size());

    // The model input size is derived from the first (sorted) YOLO output, the stride-8 head.
    const std::size_t width = yoloOutputs.front().dims[3] * 8;
    const std::size_t height = yoloOutputs.front().dims[2] * 8;

    FastsamResult result;
    result.width = width;
    result.height = height;

    const std::vector<DetectionRow> rows =
        decodeFastsamOutput(yoloOutputs, static_cast<std::int64_t>(height), static_cast<std::int64_t>(width), confThreshold, iouThreshold, numClasses);

    if(rows.empty()) {
        // The source merges a sentinel np.full((1, height, width), -1, np.int16) array with a
        // mask count of 0, producing a fully-background (255) mask.
        result.mergedMask.assign(height * width, 255);
        result.maskCount = 0;
        return result;
    }

    DAI_CHECK_V(protos.dims.size() == 4, "FastSAM protos output must be a 4D NCHW tensor, got {} dimensions.", protos.dims.size());
    DAI_CHECK_V(protos.dims[1] == protosLen, "FastSAM protos output has {} channels, expected {} prototypes.", protos.dims[1], protosLen);
    const std::size_t protoHeight = protos.dims[2];
    const std::size_t protoWidth = protos.dims[3];

    const std::vector<float> maskCoeffs = buildMaskCoeffs(rows, maskOutputs, protosLen);
    std::vector<std::uint8_t> masks = processMasks(rows, maskCoeffs, protos.values, protosLen, protoHeight, protoWidth, height, width, maskConf);
    std::size_t maskCount = rows.size();

    if(promptConfig.prompt == "bbox") {
        DAI_CHECK(promptConfig.bbox.has_value(), "FastSAMParser prompt is 'bbox' but no bounding box is set; set it with setBoundingBox().");
        masks = boxPrompt(masks, maskCount, height, width, *promptConfig.bbox);
        maskCount = 1;
    } else if(promptConfig.prompt == "point") {
        DAI_CHECK(promptConfig.points.has_value(), "FastSAMParser prompt is 'point' but no point is set; set it with setPoints().");
        DAI_CHECK(promptConfig.pointLabel.has_value(), "FastSAMParser prompt is 'point' but no point label is set; set it with setPointLabel().");
        masks = pointPrompt(masks, maskCount, height, width, *promptConfig.points, *promptConfig.pointLabel);
        maskCount = 1;
    } else {
        DAI_CHECK(promptConfig.prompt == "everything", "Prompt must be one of 'everything', 'bbox', or 'point'");
    }

    result.mergedMask = mergeMasks(masks, maskCount, height, width);
    result.maskCount = maskCount;
    return result;
}

}  // namespace FastSAMUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
