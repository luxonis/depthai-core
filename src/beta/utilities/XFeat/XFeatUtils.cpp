#include "beta/utilities/XFeat/XFeatUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "beta/utilities/MLSD/MLSDUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace XFeatUtils {

namespace {

/// Keypoint heat map NMS threshold (exclusive), hard-coded by the source detect_and_compute.
constexpr float NMS_THRESHOLD = 0.05f;
/// Keypoint heat map NMS local-maximum window size, hard-coded by the source detect_and_compute.
constexpr int NMS_KERNEL_SIZE = 5;
/// Keypoint logit softmax temperature, hard-coded by the source _get_kpts_heatmap.
constexpr float SOFTMAX_TEMPERATURE = 1.0f;
/// Cell size of the keypoint logit unfold: channel (r * 8 + c) maps to the (r, c) offset of an
/// 8x8 cell, so 64 channels produce one full-resolution cell per (height, width) grid position.
constexpr std::size_t UNFOLD_CELL = 8;

/**
 * One bilinear sampling tap: the four zero-padded, clipped source indices and their float64
 * weights. The source bilinear() computes the weights as int32 neighbor minus position, which
 * numpy promotes to float64 for both the float64 (integer candidate) and float32 (descriptor)
 * position paths.
 */
struct BilinearTap {
    std::int32_t xi0, xi1, yi0, yi1;
    double wa, wb, wc, wd;
};

/**
 * Build the bilinear tap of a normalized grid position for an image of size (h, w), porting the
 * source bilinear() with align_corners=False: the position is unnormalized to
 * ((grid + 1) * size - 1) / 2 in T precision (float64 for integer candidate positions, float32
 * for the float32 descriptor positions, following the numpy dtype promotion), the four
 * neighbors are found with floor/+1, the float64 weights are computed from the unclipped
 * neighbors, and the indices are shifted into the 1-pixel zero padding and clipped to it.
 */
template <typename T>
BilinearTap makeBilinearTap(std::int64_t h, std::int64_t w, T gridX, T gridY) {
    const T x = ((gridX + T(1)) * static_cast<T>(w) - T(1)) / T(2);
    const T y = ((gridY + T(1)) * static_cast<T>(h) - T(1)) / T(2);

    const std::int32_t x0 = static_cast<std::int32_t>(std::floor(x));
    const std::int32_t y0 = static_cast<std::int32_t>(std::floor(y));
    const std::int32_t x1 = x0 + 1;
    const std::int32_t y1 = y0 + 1;

    BilinearTap tap;
    // x1 - x is int32 minus T, promoted to float64 by numpy for both T = float and T = double.
    tap.wa = (static_cast<double>(x1) - static_cast<double>(x)) * (static_cast<double>(y1) - static_cast<double>(y));
    tap.wb = (static_cast<double>(x1) - static_cast<double>(x)) * (static_cast<double>(y) - static_cast<double>(y0));
    tap.wc = (static_cast<double>(x) - static_cast<double>(x0)) * (static_cast<double>(y1) - static_cast<double>(y));
    tap.wd = (static_cast<double>(x) - static_cast<double>(x0)) * (static_cast<double>(y) - static_cast<double>(y0));

    // Shift into the padded image and clip to it; out-of-range taps land on the zero border.
    const std::int32_t paddedW = static_cast<std::int32_t>(w) + 2;
    const std::int32_t paddedH = static_cast<std::int32_t>(h) + 2;
    tap.xi0 = std::clamp(x0 + 1, 0, paddedW - 1);
    tap.xi1 = std::clamp(x1 + 1, 0, paddedW - 1);
    tap.yi0 = std::clamp(y0 + 1, 0, paddedH - 1);
    tap.yi1 = std::clamp(y1 + 1, 0, paddedH - 1);
    return tap;
}

/**
 * Sample one channel plane of size (h, w) with a bilinear tap in float64. Taps on the zero
 * border read 0, matching the source's 1-pixel constant padding, and the weighted sum is
 * accumulated left-to-right like the numpy expression Ia*wa + Ib*wb + Ic*wc + Id*wd.
 */
double sampleWithTap(const float* channel, std::int64_t h, std::int64_t w, const BilinearTap& tap) {
    const auto fetch = [&](std::int32_t xi, std::int32_t yi) -> double {
        if(xi < 1 || xi > static_cast<std::int32_t>(w) || yi < 1 || yi > static_cast<std::int32_t>(h)) {
            return 0.0;
        }
        return static_cast<double>(channel[static_cast<std::size_t>(yi - 1) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xi - 1)]);
    };
    return ((fetch(tap.xi0, tap.yi0) * tap.wa + fetch(tap.xi0, tap.yi1) * tap.wb) + fetch(tap.xi1, tap.yi0) * tap.wc) + fetch(tap.xi1, tap.yi1) * tap.wd;
}

/**
 * Validate one XFeat tensor: 4D NCHW with batch size 1, at least minChannels channels and a
 * value count matching the shape.
 */
void validateTensor(const XFeatTensor& tensor, const char* name, std::size_t minChannels, std::size_t maxChannels) {
    DAI_CHECK_V(tensor.dims.size() == 4, "XFeat: expected a 4D (batch, channels, height, width) {} tensor, got {} dimensions.", name, tensor.dims.size());
    DAI_CHECK_V(tensor.dims[0] == 1, "XFeat: expected a {} tensor with batch size 1, got batch size {}.", name, tensor.dims[0]);
    DAI_CHECK_V(tensor.dims[1] >= minChannels && tensor.dims[1] <= maxChannels,
                "XFeat: expected a {} tensor with {} to {} channels, got {} channels.",
                name,
                minChannels,
                maxChannels,
                tensor.dims[1]);
    DAI_CHECK_V(tensor.values.size() == tensor.dims[0] * tensor.dims[1] * tensor.dims[2] * tensor.dims[3],
                "XFeat: cannot interpret {} {} tensor values as shape ({}, {}, {}, {}).",
                tensor.values.size(),
                name,
                tensor.dims[0],
                tensor.dims[1],
                tensor.dims[2],
                tensor.dims[3]);
}

/// numpy maximum fold: the accumulated maximum propagates NaN from either operand.
template <typename T>
inline T nanPropagatingMax(T acc, T value) {
    if(std::isnan(acc)) {
        return acc;
    }
    if(std::isnan(value)) {
        return value;
    }
    return std::max(acc, value);
}

/**
 * numpy floating-point argmax over count values with the given stride: first occurrence of the
 * maximum, where a NaN is maximal (the first NaN wins).
 */
template <typename T>
std::size_t numpyArgmax(const T* values, std::size_t count, std::size_t stride) {
    std::size_t best = 0;
    T maxValue = values[0];
    if(std::isnan(maxValue)) {
        return 0;
    }
    for(std::size_t i = 1; i < count; i++) {
        const T value = values[i * stride];
        if(!(value <= maxValue)) {
            maxValue = value;
            best = i;
            if(std::isnan(maxValue)) {
                break;
            }
        }
    }
    return best;
}

}  // namespace

XFeatTensors extractXFeatTensors(dai::NNData& nnData, const std::string& featsName, const std::string& keypointsName, const std::string& heatmapsName) {
#ifdef DEPTHAI_XTENSOR_SUPPORT
    const auto get = [&nnData](const std::string& name) {
        xt::xarray<float> tensor = nnData.getTensor<float>(name, TensorInfo::StorageOrder::NCHW, true);
        XFeatTensor result;
        result.dims.assign(tensor.shape().begin(), tensor.shape().end());
        result.values.assign(tensor.begin(), tensor.end());
        return result;
    };
    XFeatTensors tensors;
    tensors.feats = get(featsName);
    tensors.keypoints = get(keypointsName);
    tensors.heatmaps = get(heatmapsName);
    return tensors;
#else
    (void)nnData;
    (void)featsName;
    (void)keypointsName;
    (void)heatmapsName;
    throw std::runtime_error(
        "XFeat parsers require xtensor support to convert the model output tensors to NCHW orientation, but xtensor support is not available.");
#endif
}

std::optional<XFeatResult> detectAndCompute(const XFeatTensor& feats,
                                            const XFeatTensor& keypoints,
                                            const XFeatTensor& heatmaps,
                                            double resizeRateW,
                                            double resizeRateH,
                                            std::uint32_t inputWidth,
                                            std::uint32_t inputHeight,
                                            int topK) {
    constexpr std::size_t noLimit = std::numeric_limits<std::size_t>::max();
    validateTensor(feats, "feats", 1, noLimit);
    // The keypoint logit unfold keeps the first 64 channels; the softmax runs over all of them.
    validateTensor(keypoints, "keypoints", UNFOLD_CELL * UNFOLD_CELL, noLimit);
    // The candidate scores multiply one reliability value per candidate, so the reliability heat
    // map must have exactly one channel.
    validateTensor(heatmaps, "heatmaps", 1, 1);

    const std::size_t descriptorSize = feats.dims[1];
    const std::size_t featsH = feats.dims[2];
    const std::size_t featsW = feats.dims[3];
    const std::size_t featsPlane = featsH * featsW;

    // feats = feats / np.linalg.norm(feats, axis=1, keepdims=True): the squares are accumulated
    // sequentially over the channel axis in float32, matching numpy's strided-axis reduce.
    std::vector<float> normalizedFeats(feats.values);
    {
        std::vector<float> squaredSum(featsPlane, 0.0f);
        for(std::size_t c = 0; c < descriptorSize; c++) {
            const float* channel = feats.values.data() + c * featsPlane;
            for(std::size_t i = 0; i < featsPlane; i++) {
                squaredSum[i] += channel[i] * channel[i];
            }
        }
        for(std::size_t i = 0; i < featsPlane; i++) {
            squaredSum[i] = std::sqrt(squaredSum[i]);
        }
        for(std::size_t c = 0; c < descriptorSize; c++) {
            float* channel = normalizedFeats.data() + c * featsPlane;
            for(std::size_t i = 0; i < featsPlane; i++) {
                channel[i] /= squaredSum[i];
            }
        }
    }

    // _get_kpts_heatmap: per-cell softmax over all channels, then the first 64 channels are
    // unfolded into the full-resolution keypoint heat map of shape (kptsH * 8, kptsW * 8), where
    // position (y, x) reads softmax channel (y % 8) * 8 + (x % 8) at grid cell (y / 8, x / 8).
    const std::size_t kptsChannels = keypoints.dims[1];
    const std::size_t kptsH = keypoints.dims[2];
    const std::size_t kptsW = keypoints.dims[3];
    const std::size_t kptsPlane = kptsH * kptsW;
    const std::size_t heatH = kptsH * UNFOLD_CELL;
    const std::size_t heatW = kptsW * UNFOLD_CELL;
    std::vector<float> keypointHeat(heatH * heatW);
    {
        std::vector<float> exponentials(kptsChannels * kptsPlane);
        std::vector<float> expSum(kptsPlane, 0.0f);
        // The exponential sum accumulates channel 0, 1, ... in ascending order for every grid
        // position, matching numpy's sequential strided-axis reduce in float32.
        for(std::size_t c = 0; c < kptsChannels; c++) {
            const float* channel = keypoints.values.data() + c * kptsPlane;
            float* expChannel = exponentials.data() + c * kptsPlane;
            for(std::size_t i = 0; i < kptsPlane; i++) {
                expChannel[i] = std::exp(channel[i] * SOFTMAX_TEMPERATURE);
                expSum[i] += expChannel[i];
            }
        }
        for(std::size_t y = 0; y < heatH; y++) {
            const std::size_t channel = (y % UNFOLD_CELL) * UNFOLD_CELL;
            const std::size_t rowOffset = (y / UNFOLD_CELL) * kptsW;
            for(std::size_t x = 0; x < heatW; x++) {
                const std::size_t gridIndex = rowOffset + x / UNFOLD_CELL;
                keypointHeat[y * heatW + x] = exponentials[(channel + x % UNFOLD_CELL) * kptsPlane + gridIndex] / expSum[gridIndex];
            }
        }
    }

    // _nms: candidates are local maxima of the keypoint heat map over a 5x5 zero-padded window
    // with a value strictly above the threshold, in row-major scan order as (x, y).
    std::vector<std::array<std::int64_t, 2>> candidates;
    {
        const int pad = NMS_KERNEL_SIZE / 2;
        for(std::int64_t y = 0; y < static_cast<std::int64_t>(heatH); y++) {
            for(std::int64_t x = 0; x < static_cast<std::int64_t>(heatW); x++) {
                const float value = keypointHeat[y * heatW + x];
                // np.max over the window propagates NaN; a NaN local maximum never equals value.
                float windowMax = -std::numeric_limits<float>::infinity();
                for(std::int64_t dy = -pad; dy <= pad; dy++) {
                    for(std::int64_t dx = -pad; dx <= pad; dx++) {
                        const std::int64_t ny = y + dy;
                        const std::int64_t nx = x + dx;
                        const bool inBounds = ny >= 0 && ny < static_cast<std::int64_t>(heatH) && nx >= 0 && nx < static_cast<std::int64_t>(heatW);
                        windowMax = nanPropagatingMax(windowMax, inBounds ? keypointHeat[ny * heatW + nx] : 0.0f);
                    }
                }
                if(value == windowMax && value > NMS_THRESHOLD) {
                    candidates.push_back({x, y});
                }
            }
        }
    }

    // if mkpts.size == 0: return None
    if(candidates.empty()) {
        return std::nullopt;
    }

    // Candidate scores: the keypoint heat map and the reliability heat map are bilinearly
    // sampled at the integer candidate positions. Integer positions divided by the grid extent
    // promote to float64 in the source normgrid(), so this path computes in float64 and the
    // product is cast to float32, matching scores.astype(np.float32).
    const std::size_t heatmapsH = heatmaps.dims[2];
    const std::size_t heatmapsW = heatmaps.dims[3];
    std::vector<float> scores(candidates.size());
    for(std::size_t i = 0; i < candidates.size(); i++) {
        const double gridX = 2.0 * (static_cast<double>(candidates[i][0]) / static_cast<double>(inputWidth - 1)) - 1.0;
        const double gridY = 2.0 * (static_cast<double>(candidates[i][1]) / static_cast<double>(inputHeight - 1)) - 1.0;
        const auto keypointTap = makeBilinearTap<double>(static_cast<std::int64_t>(heatH), static_cast<std::int64_t>(heatW), gridX, gridY);
        const double keypointValue = sampleWithTap(keypointHeat.data(), static_cast<std::int64_t>(heatH), static_cast<std::int64_t>(heatW), keypointTap);
        const auto reliabilityTap = makeBilinearTap<double>(static_cast<std::int64_t>(heatmapsH), static_cast<std::int64_t>(heatmapsW), gridX, gridY);
        const double reliabilityValue =
            sampleWithTap(heatmaps.values.data(), static_cast<std::int64_t>(heatmapsH), static_cast<std::int64_t>(heatmapsW), reliabilityTap);
        scores[i] = static_cast<float>(keypointValue * reliabilityValue);
        // scores[np.all(mkpts == 0, axis=-1)] = -1
        if(candidates[i][0] == 0 && candidates[i][1] == 0) {
            scores[i] = -1.0f;
        }
    }

    // idxs = np.argsort(-scores), then keep the first topK sorted candidates. A negative topK
    // follows Python slice semantics like the source's [:, :top_k].
    const auto order = MLSDUtils::argsortDescending(scores);
    const std::int64_t total = static_cast<std::int64_t>(order.size());
    const std::int64_t kept = topK < 0 ? std::max<std::int64_t>(std::int64_t(0), total + topK) : std::min<std::int64_t>(static_cast<std::int64_t>(topK), total);

    XFeatResult result;
    result.descriptorSize = descriptorSize;
    result.keypoints.reserve(kept);
    result.scores.reserve(kept);
    result.descriptors.reserve(kept * descriptorSize);
    std::vector<double> descriptor(descriptorSize);
    for(std::int64_t rank = 0; rank < kept; rank++) {
        const std::size_t index = static_cast<std::size_t>(order[rank]);
        const float score = scores[index];
        // valid = scores > 0
        if(!(score > 0.0f)) {
            continue;
        }

        // The kept positions are cast to float32 before the descriptor sampling, so this path
        // computes the grid normalization and unnormalized positions in float32 like the
        // source; the weights and the weighted sum promote to float64.
        const float posX = static_cast<float>(candidates[index][0]);
        const float posY = static_cast<float>(candidates[index][1]);
        const float gridX = 2.0f * (posX / static_cast<float>(inputWidth - 1)) - 1.0f;
        const float gridY = 2.0f * (posY / static_cast<float>(inputHeight - 1)) - 1.0f;
        const auto tap = makeBilinearTap<float>(static_cast<std::int64_t>(featsH), static_cast<std::int64_t>(featsW), gridX, gridY);
        double squaredSum = 0.0;
        for(std::size_t c = 0; c < descriptorSize; c++) {
            descriptor[c] = sampleWithTap(normalizedFeats.data() + c * featsPlane, static_cast<std::int64_t>(featsH), static_cast<std::int64_t>(featsW), tap);
            squaredSum += descriptor[c] * descriptor[c];
        }
        // Descriptor L2 normalization: numpy reduces this norm sequentially as well because the
        // squared array inherits the transposed layout, keeping the channel axis strided.
        const double norm = std::sqrt(squaredSum);
        for(std::size_t c = 0; c < descriptorSize; c++) {
            result.descriptors.push_back(descriptor[c] / norm);
        }

        // mkpts *= np.array([resize_rate_w, resize_rate_h]): float32 *= float64 computes in
        // float64 and casts back to float32.
        result.keypoints.push_back({static_cast<float>(static_cast<double>(posX) * resizeRateW), static_cast<float>(static_cast<double>(posY) * resizeRateH)});
        result.scores.push_back(score);
    }
    return result;
}

MatchedPoints matchResults(const XFeatResult& reference, const XFeatResult& target, double minCossim) {
    const std::size_t referenceCount = reference.keypoints.size();
    const std::size_t targetCount = target.keypoints.size();
    // The source _match_mkpts fails inside np.argmax on an empty similarity axis whenever either
    // result holds no keypoints.
    DAI_CHECK_V(referenceCount > 0 && targetCount > 0,
                "XFeat: cannot match results with no keypoints, got {} reference keypoints and {} target keypoints.",
                referenceCount,
                targetCount);
    DAI_CHECK_V(reference.descriptorSize == target.descriptorSize,
                "XFeat: cannot match results with different descriptor sizes, got {} and {}.",
                reference.descriptorSize,
                target.descriptorSize);
    const std::size_t descriptorSize = reference.descriptorSize;

    // cossim = feats1 @ feats2.T in float64 (the descriptors are float64). The transposed
    // product feats2 @ feats1.T holds the same values, so the column argmax below reproduces
    // match21.
    std::vector<double> cossim(referenceCount * targetCount);
    for(std::size_t i = 0; i < referenceCount; i++) {
        const double* referenceDescriptor = reference.descriptors.data() + i * descriptorSize;
        for(std::size_t j = 0; j < targetCount; j++) {
            const double* targetDescriptor = target.descriptors.data() + j * descriptorSize;
            double dot = 0.0;
            for(std::size_t c = 0; c < descriptorSize; c++) {
                dot += referenceDescriptor[c] * targetDescriptor[c];
            }
            cossim[i * targetCount + j] = dot;
        }
    }

    // match12 = np.argmax(cossim, axis=1), match21 = np.argmax(cossim_t, axis=1)
    std::vector<std::size_t> match12(referenceCount);
    for(std::size_t i = 0; i < referenceCount; i++) {
        match12[i] = numpyArgmax(cossim.data() + i * targetCount, targetCount, 1);
    }
    std::vector<std::size_t> match21(targetCount);
    for(std::size_t j = 0; j < targetCount; j++) {
        match21[j] = numpyArgmax(cossim.data() + j, referenceCount, targetCount);
    }

    MatchedPoints matchedPoints;
    for(std::size_t i = 0; i < referenceCount; i++) {
        // mutual = match21[match12] == np.arange(len(match12))
        if(match21[match12[i]] != i) {
            continue;
        }
        if(minCossim > 0) {
            // max_cossim = np.max(cossim, axis=1); good = max_cossim > min_cossim
            double maxCossim = cossim[i * targetCount];
            for(std::size_t j = 1; j < targetCount; j++) {
                maxCossim = nanPropagatingMax(maxCossim, cossim[i * targetCount + j]);
            }
            if(!(maxCossim > minCossim)) {
                continue;
            }
        }
        matchedPoints.referencePoints.push_back(reference.keypoints[i]);
        matchedPoints.targetPoints.push_back(target.keypoints[match12[i]]);
    }
    return matchedPoints;
}

std::shared_ptr<dai::TrackedFeatures> createTrackedFeaturesMessage(const MatchedPoints& matchedPoints) {
    DAI_CHECK(matchedPoints.referencePoints.size() == matchedPoints.targetPoints.size(),
              "The number of reference points and target points should be the same.");

    auto message = std::make_shared<dai::TrackedFeatures>();
    message->trackedFeatures.reserve(2 * matchedPoints.referencePoints.size());
    for(std::size_t i = 0; i < matchedPoints.referencePoints.size(); i++) {
        TrackedFeature referenceFeature;
        referenceFeature.position.x = matchedPoints.referencePoints[i][0];
        referenceFeature.position.y = matchedPoints.referencePoints[i][1];
        referenceFeature.id = static_cast<std::uint32_t>(i);
        referenceFeature.age = 0;
        message->trackedFeatures.push_back(referenceFeature);

        TrackedFeature targetFeature;
        targetFeature.position.x = matchedPoints.targetPoints[i][0];
        targetFeature.position.y = matchedPoints.targetPoints[i][1];
        targetFeature.id = static_cast<std::uint32_t>(i);
        targetFeature.age = 1;
        message->trackedFeatures.push_back(targetFeature);
    }
    return message;
}

}  // namespace XFeatUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
