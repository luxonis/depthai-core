#include "beta/utilities/MLSD/MLSDUtils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "utility/ErrorMacros.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace MLSDUtils {

namespace {

/*
 * The M-LSD top-k candidate selection replicates numpy's
 * np.argpartition(heat, -topk)[-topk:] followed by np.argsort(-scores), including the order of
 * equal scores. The two helpers below are C++ ports of NumPy's generic (portable scalar)
 * argpartition (introselect) and argsort (introsort) algorithms, taken from
 * numpy/_core/src/npysort/{selection.cpp, quicksort.hpp, npysort_heapsort.h} and
 * numpy/_core/src/common/numpy_tag.h at tag v2.5.1.
 *
 * NumPy is licensed under the BSD 3-Clause License:
 *   Copyright (c) 2005-2025, NumPy Developers. All rights reserved.
 * The quickselect is loosely based on the public domain implementation by Nicolas Devillard
 * (http://ndevilla.free.fr/median/median/) and the sorting code is due to Charles R. Harris for
 * numarray. See https://github.com/numpy/numpy/blob/v2.5.1/LICENSE.txt for the full license
 * text; redistribution of this derived code retains the above copyright notice per the license.
 *
 * Note: recent numpy builds dispatch argsort/argpartition of 32-bit dtypes to CPU-specific
 * x86-simd-sort kernels on AVX2/AVX-512 hardware, whose ordering of equal values differs per
 * CPU. The generic scalar algorithms ported here are numpy's portable semantics used on all
 * other hardware.
 */

/// numpy floating_point_type::less: NaN compares as the largest value (sorts to the end).
inline bool floatLess(float a, float b) {
    return a < b || (b != b && a == a);
}

/// numpy npy_get_msb: index of the most significant set bit, 0 for num == 0.
inline int npyGetMsb(std::int64_t num) {
    int msb = 0;
    while(num >>= 1) {
        msb++;
    }
    return msb;
}

constexpr std::int64_t SMALL_QUICKSORT = 15;

/// numpy npysort_heapsort.h aheapsort_: indirect heapsort of tosort[0..n) ascending by v.
void argHeapSort(const float* v, std::int64_t* tosort, std::int64_t n) {
    // The array is offset by one for heapsort indexing.
    std::int64_t* a = tosort - 1;

    for(std::int64_t l = n >> 1; l > 0; --l) {
        const std::int64_t tmp = a[l];
        std::int64_t i = l;
        std::int64_t j = l << 1;
        while(j <= n) {
            if(j < n && floatLess(v[a[j]], v[a[j + 1]])) {
                j += 1;
            }
            if(floatLess(v[tmp], v[a[j]])) {
                a[i] = a[j];
                i = j;
                j += j;
            } else {
                break;
            }
        }
        a[i] = tmp;
    }

    for(; n > 1;) {
        const std::int64_t tmp = a[n];
        a[n] = a[1];
        n -= 1;
        std::int64_t i = 1;
        std::int64_t j = 2;
        while(j <= n) {
            if(j < n && floatLess(v[a[j]], v[a[j + 1]])) {
                j++;
            }
            if(floatLess(v[tmp], v[a[j]])) {
                a[i] = a[j];
                i = j;
                j += j;
            } else {
                break;
            }
        }
        a[i] = tmp;
    }
}

/// numpy quicksort.hpp aquicksort_: indirect introsort of tosort[0..num) ascending by v.
void argQuickSort(const float* v, std::int64_t* tosort, std::int64_t num) {
    if(num < 1) {
        return;
    }
    std::int64_t* pl = tosort;
    std::int64_t* pr = tosort + num - 1;
    std::vector<std::pair<std::int64_t*, std::int64_t*>> stack;
    std::vector<int> depth;
    int cdepth = npyGetMsb(num) * 2;

    for(;;) {
        if(cdepth < 0) {
            argHeapSort(v, pl, pr - pl + 1);
            // stack pop
            if(stack.empty()) {
                break;
            }
            std::tie(pl, pr) = stack.back();
            stack.pop_back();
            cdepth = depth.back();
            depth.pop_back();
            continue;
        }
        while((pr - pl) > SMALL_QUICKSORT) {
            // quicksort partition with median-of-3 pivot
            std::int64_t* pm = pl + ((pr - pl) >> 1);
            if(floatLess(v[*pm], v[*pl])) {
                std::swap(*pm, *pl);
            }
            if(floatLess(v[*pr], v[*pm])) {
                std::swap(*pr, *pm);
            }
            if(floatLess(v[*pm], v[*pl])) {
                std::swap(*pm, *pl);
            }
            const float vp = v[*pm];
            std::int64_t* pi = pl;
            std::int64_t* pj = pr - 1;
            std::swap(*pm, *pj);
            for(;;) {
                do {
                    ++pi;
                } while(floatLess(v[*pi], vp));
                do {
                    --pj;
                } while(floatLess(vp, v[*pj]));
                if(pi >= pj) {
                    break;
                }
                std::swap(*pi, *pj);
            }
            std::int64_t* pk = pr - 1;
            std::swap(*pi, *pk);
            // push largest partition on stack
            if(pi - pl < pr - pi) {
                stack.emplace_back(pi + 1, pr);
                pr = pi - 1;
            } else {
                stack.emplace_back(pl, pi - 1);
                pl = pi + 1;
            }
            depth.push_back(--cdepth);
        }

        // insertion sort
        for(std::int64_t* pi = pl + 1; pi <= pr; ++pi) {
            const std::int64_t vi = *pi;
            const float vp = v[vi];
            std::int64_t* pj = pi;
            std::int64_t* pk = pi - 1;
            while(pj > pl && floatLess(vp, v[*pk])) {
                *pj-- = *pk--;
            }
            *pj = vi;
        }
        if(stack.empty()) {
            break;
        }
        std::tie(pl, pr) = stack.back();
        stack.pop_back();
        cdepth = depth.back();
        depth.pop_back();
    }
}

std::int64_t argIntroSelect(const float* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth);

/// numpy selection.cpp median3_swap_: moves the median-of-3 pivot to low and the 3-lowest
/// element to low + 1 for the unguarded partition.
void argMedian3Swap(const float* v, std::int64_t* tosort, std::int64_t low, std::int64_t mid, std::int64_t high) {
    if(floatLess(v[tosort[high]], v[tosort[mid]])) {
        std::swap(tosort[high], tosort[mid]);
    }
    if(floatLess(v[tosort[high]], v[tosort[low]])) {
        std::swap(tosort[high], tosort[low]);
    }
    // move pivot to low
    if(floatLess(v[tosort[low]], v[tosort[mid]])) {
        std::swap(tosort[low], tosort[mid]);
    }
    // move 3-lowest element to low + 1
    std::swap(tosort[mid], tosort[low + 1]);
}

/// numpy selection.cpp median5_: selects the index of the median of five elements.
std::int64_t argMedian5(const float* v, std::int64_t* tosort) {
    if(floatLess(v[tosort[1]], v[tosort[0]])) {
        std::swap(tosort[1], tosort[0]);
    }
    if(floatLess(v[tosort[4]], v[tosort[3]])) {
        std::swap(tosort[4], tosort[3]);
    }
    if(floatLess(v[tosort[3]], v[tosort[0]])) {
        std::swap(tosort[3], tosort[0]);
    }
    if(floatLess(v[tosort[4]], v[tosort[1]])) {
        std::swap(tosort[4], tosort[1]);
    }
    if(floatLess(v[tosort[2]], v[tosort[1]])) {
        std::swap(tosort[2], tosort[1]);
    }
    if(floatLess(v[tosort[3]], v[tosort[2]])) {
        if(floatLess(v[tosort[3]], v[tosort[1]])) {
            return 1;
        } else {
            return 3;
        }
    } else {
        // tosort[1] and tosort[2] swapped into order above
        return 2;
    }
}

/// numpy selection.cpp median_of_median5_: median of medians of blocks of 5, guaranteeing a
/// 30%/70% split for the linear-time worst case.
std::int64_t argMedianOfMedian5(const float* v, std::int64_t* tosort, std::int64_t num) {
    const std::int64_t right = num - 1;
    const std::int64_t nmed = (right + 1) / 5;
    std::int64_t subleft = 0;
    for(std::int64_t i = 0; i < nmed; i++, subleft += 5) {
        const std::int64_t m = argMedian5(v, tosort + subleft);
        std::swap(tosort[subleft + m], tosort[i]);
    }

    if(nmed > 2) {
        argIntroSelect(v, tosort, nmed, nmed / 2);
    }
    return nmed / 2;
}

/// numpy selection.cpp dumb_select_: N^2 selection, fast only for very small kth.
void argDumbSelect(const float* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth) {
    for(std::int64_t i = 0; i <= kth; i++) {
        std::int64_t minidx = i;
        float minval = v[tosort[i]];
        for(std::int64_t k = i + 1; k < num; k++) {
            if(floatLess(v[tosort[k]], minval)) {
                minidx = k;
                minval = v[tosort[k]];
            }
        }
        std::swap(tosort[i], tosort[minidx]);
    }
}

/// numpy selection.cpp unguarded_partition_: partitions around the pivot; the median-of-3
/// preparation removes the need for bound checks.
void argUnguardedPartition(const float* v, std::int64_t* tosort, const float pivot, std::int64_t* ll, std::int64_t* hh) {
    for(;;) {
        do {
            (*ll)++;
        } while(floatLess(v[tosort[*ll]], pivot));
        do {
            (*hh)--;
        } while(floatLess(pivot, v[tosort[*hh]]));

        if(*hh < *ll) {
            break;
        }

        std::swap(tosort[*ll], tosort[*hh]);
    }
}

/// numpy selection.cpp introselect_ (arg variant, single kth): median-of-3 quickselect with a
/// cutoff to median-of-medians-of-5. The numpy pivot stack is omitted: with a single kth per
/// call it is written but never read within the call.
std::int64_t argIntroSelect(const float* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth) {
    std::int64_t low = 0;
    std::int64_t high = num - 1;

    // use a faster O(n*kth) algorithm for very small kth
    if(kth - low < 3) {
        argDumbSelect(v, tosort + low, high - low + 1, kth - low);
        return 0;
    }
    // useful to check if NaN present via partition(d, (x, -1))
    else if(kth == num - 1) {
        std::int64_t maxidx = low;
        float maxval = v[tosort[low]];
        for(std::int64_t k = low + 1; k < num; k++) {
            if(!floatLess(v[tosort[k]], maxval)) {
                maxidx = k;
                maxval = v[tosort[k]];
            }
        }
        std::swap(tosort[kth], tosort[maxidx]);
        return 0;
    }

    int depthLimit = npyGetMsb(num) * 2;

    // guarantee three elements
    for(; low + 1 < high;) {
        std::int64_t ll = low + 1;
        std::int64_t hh = high;

        /*
         * if we aren't making sufficient progress with median of 3 fall back to
         * median-of-median5 pivot for linear worst case; med3 for small sizes is required to do
         * unguarded partition
         */
        if(depthLimit > 0 || hh - ll < 5) {
            const std::int64_t mid = low + (high - low) / 2;
            // median of 3 pivot strategy, swapping for efficient partition
            argMedian3Swap(v, tosort, low, mid, high);
        } else {
            const std::int64_t mid = ll + argMedianOfMedian5(v, tosort + ll, hh - ll);
            std::swap(tosort[mid], tosort[low]);
            // adapt for the larger partition than med3 pivot
            ll--;
            hh++;
        }

        depthLimit--;

        /*
         * find place to put pivot (in low): previous swapping removes need for bound checks
         * pivot 3-lowest [x x x] 3-highest
         */
        argUnguardedPartition(v, tosort, v[tosort[low]], &ll, &hh);

        // move pivot into position
        std::swap(tosort[low], tosort[hh]);

        if(hh >= kth) {
            high = hh - 1;
        }
        if(hh <= kth) {
            low = ll;
        }
    }

    // two elements
    if(high == low + 1) {
        if(floatLess(v[tosort[high]], v[tosort[low]])) {
            std::swap(tosort[high], tosort[low]);
        }
    }

    return 0;
}

}  // namespace

std::vector<std::int64_t> topKIndicesByScore(const std::vector<float>& heatValues, int topK) {
    DAI_CHECK(topK > 0, "topk_n must be a positive integer.");
    DAI_CHECK(!heatValues.empty(), "Expected a non-empty heat tensor.");

    const std::int64_t num = static_cast<std::int64_t>(heatValues.size());
    // The number of candidates is capped at the heat map size.
    const std::int64_t topk = std::min<std::int64_t>(static_cast<std::int64_t>(topK), num);
    const std::int64_t kth = num - topk;

    // indices_np = np.argpartition(heat_flat, -topk_n)[-topk_n:]
    std::vector<std::int64_t> tosort(num);
    std::iota(tosort.begin(), tosort.end(), 0);
    argIntroSelect(heatValues.data(), tosort.data(), num, kth);

    // sorted_idx = indices_np[np.argsort(-heat_flat[indices_np])]
    const std::int64_t* indices = tosort.data() + kth;
    std::vector<float> negScores(topk);
    for(std::int64_t i = 0; i < topk; i++) {
        negScores[i] = -heatValues[indices[i]];
    }
    std::vector<std::int64_t> order(topk);
    std::iota(order.begin(), order.end(), 0);
    argQuickSort(negScores.data(), order.data(), topk);

    std::vector<std::int64_t> sortedIndices(topk);
    for(std::int64_t i = 0; i < topk; i++) {
        sortedIndices[i] = indices[order[i]];
    }
    return sortedIndices;
}

MlsdLines computeMlsdLines(const std::vector<float>& tpMapValues,
                           const std::vector<std::size_t>& tpMapDims,
                           const std::vector<float>& heatValues,
                           int topK,
                           float scoreThreshold,
                           float distanceThreshold,
                           std::uint32_t inputWidth,
                           std::uint32_t inputHeight) {
    DAI_CHECK(tpMapDims.size() == 4, "Invalid shape of the tpMap tensor. Should be 4D.");
    const std::size_t batch = tpMapDims[0];
    const std::size_t channels = tpMapDims[1];
    const std::size_t height = tpMapDims[2];
    const std::size_t width = tpMapDims[3];
    DAI_CHECK_V(batch >= 1, "Expected a tpMap tensor with at least one batch entry, got batch size {}.", batch);
    // Channel 0 is the unused center map; channels 1 to 4 are the displacement maps.
    DAI_CHECK_V(channels >= 5, "Expected a tpMap tensor with at least 5 channels (1 center + 4 displacement), got {} channels.", channels);
    DAI_CHECK_V(tpMapValues.size() == batch * channels * height * width,
                "Cannot interpret {} tpMap values as shape ({}, {}, {}, {}).",
                tpMapValues.size(),
                batch,
                channels,
                height,
                width);
    DAI_CHECK(inputWidth > 0 && inputHeight > 0, "Input size must be greater than 0.");

    const auto sortedIndices = topKIndicesByScore(heatValues, topK);

    // Displacement maps of the first batch entry: vmap[y][x][c] = tpMap[0][1 + c][y][x].
    const std::size_t mapSize = height * width;
    const float* displacement = tpMapValues.data() + mapSize;

    MlsdLines result;
    for(const std::int64_t index : sortedIndices) {
        const float score = heatValues[index];
        // yy, xx = np.divmod(sorted_idx, w)
        const std::int64_t y = index / static_cast<std::int64_t>(width);
        const std::int64_t x = index % static_cast<std::int64_t>(width);
        // Equivalent of the numpy IndexError when a heat position falls outside the tpMap grid.
        DAI_CHECK_V(
            y < static_cast<std::int64_t>(height), "Heat map index {} maps to row {} outside the tpMap spatial shape ({}, {}).", index, y, height, width);

        const std::size_t offset = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
        const float dispStartX = displacement[offset];
        const float dispStartY = displacement[mapSize + offset];
        const float dispEndX = displacement[2 * mapSize + offset];
        const float dispEndY = displacement[3 * mapSize + offset];

        // start = (x, y) + displacement start, end = (x, y) + displacement end, computed in
        // float64 like the numpy int64 + float32 promotion.
        const double startX = static_cast<double>(x) + static_cast<double>(dispStartX);
        const double startY = static_cast<double>(y) + static_cast<double>(dispStartY);
        const double endX = static_cast<double>(x) + static_cast<double>(dispEndX);
        const double endY = static_cast<double>(y) + static_cast<double>(dispEndY);

        const double distance = std::sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));

        // keep = (pts_score > score_thr) & (dists > dist_thr); both thresholds are exclusive.
        // The score comparison is in float32 and the distance comparison in float64, matching
        // the numpy promotion rules.
        if(!(score > scoreThreshold && distance > static_cast<double>(distanceThreshold))) {
            continue;
        }

        // lines = 2 * lines / input_size: cast to float32, scale by 2 (the heat map grid is
        // half the model input resolution) and normalize, x by width and y by height. The
        // source hard-codes a single scalar input size (512 for all known M-LSD models); the
        // per-axis normalization generalizes it to non-square inputs.
        const std::array<float, 4> line{(2.0f * static_cast<float>(startX)) / static_cast<float>(inputWidth),
                                        (2.0f * static_cast<float>(startY)) / static_cast<float>(inputHeight),
                                        (2.0f * static_cast<float>(endX)) / static_cast<float>(inputWidth),
                                        (2.0f * static_cast<float>(endY)) / static_cast<float>(inputHeight)};
        result.lines.push_back(line);
        result.scores.push_back(score);
    }
    return result;
}

std::shared_ptr<Lines> createLinesMessage(const MlsdLines& mlsdLines) {
    DAI_CHECK_V(mlsdLines.scores.size() == mlsdLines.lines.size(),
                "Scores should have same length as lines, got {} and {}.",
                mlsdLines.scores.size(),
                mlsdLines.lines.size());

    auto message = std::make_shared<Lines>();
    message->lines.reserve(mlsdLines.lines.size());
    for(std::size_t i = 0; i < mlsdLines.lines.size(); i++) {
        const auto& coordinates = mlsdLines.lines[i];
        Line line;
        line.startPoint = Point2f(coordinates[0], coordinates[1]);
        line.endPoint = Point2f(coordinates[2], coordinates[3]);

        double confidence = static_cast<double>(mlsdLines.scores[i]);
        DAI_CHECK(!(confidence < -0.1 || confidence > 1.1), "Confidence must be between 0 and 1.");
        if(!(confidence >= 0.0 && confidence <= 1.0)) {
            confidence = std::max(0.0, std::min(1.0, confidence));
            logger::info("Confidence value was clipped to [0, 1].");
        }
        line.confidence = static_cast<float>(confidence);
        message->lines.push_back(line);
    }
    return message;
}

}  // namespace MLSDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
