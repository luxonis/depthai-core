#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iterator>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "depthai/utility/ImageAlignDepth.hpp"

namespace dai::detail {

// Cache element offsets, including the source stride; -1 denotes the constant-zero border.
inline std::vector<std::ptrdiff_t> nearestDepthOffsets(const cv::Mat& mapX, const cv::Mat& mapY, cv::Size sourceSize, size_t sourceStride) {
    CV_Assert(mapX.type() == CV_32FC1 && mapY.type() == CV_32FC1 && mapX.size() == mapY.size());
    std::vector<std::ptrdiff_t> offsets(mapX.total(), -1);
    for(int y = 0; y < mapX.rows; ++y) {
        for(int x = 0; x < mapX.cols; ++x) {
            const float u = mapX.at<float>(y, x), v = mapY.at<float>(y, x);
            if(!std::isfinite(u) || !std::isfinite(v) || u < -1 || v < -1 || u > sourceSize.width || v > sourceSize.height) continue;
            const int col = cvRound(u), row = cvRound(v);
            if(col >= 0 && row >= 0 && col < sourceSize.width && row < sourceSize.height) offsets[y * mapX.cols + x] = row * sourceStride + col;
        }
    }
    return offsets;
}

struct ImageAlignCpuParallelFor {
    template <typename Body>
    void operator()(const cv::Range& range, const Body& body, double stripes) const {
        cv::parallel_for_(range, body, stripes);
    }
};

// Produce the frame with gpu if there is one. If runGpu throws, report the error, release the GPU
// (so later frames stay on the CPU) and produce this frame with runCpu. Returns whether the GPU produced it.
template <typename Gpu, typename GpuRun, typename CpuRun, typename OnError>
inline bool runGpuWithCpuFallback(std::unique_ptr<Gpu>& gpu, GpuRun&& runGpu, CpuRun&& runCpu, OnError&& onError) {
    if(gpu) {
        try {
            runGpu(*gpu);
            return true;
        } catch(const std::exception& error) {
            gpu.reset();  // Release first, so a throwing onError cannot leave the failed GPU in use.
            onError(error);
        }
    }
    runCpu();
    return false;
}

// Number of entries in a depth-shift table: one per RAW16 depth value.
inline constexpr size_t kDepthShiftLutSize = 65536;

// Fill lut[d] = shift / d for every RAW16 depth d; lut[0] stays 0 for invalid depth.
template <typename DepthLut>
inline void fillDepthShiftLut(DepthLut& lut, float shift) {
    CV_Assert(std::size(lut) == kDepthShiftLutSize);
    lut[0] = 0.0f;
    for(size_t depth = 1; depth < kDepthShiftLutSize; ++depth) lut[depth] = shift / static_cast<float>(depth);
}

struct DepthAlignTables {
    std::vector<float> rectifiedRayZ;       // Source Z per unit rectified depth, for each rectified pixel
    std::vector<float> alignedDepthScales;  // Target Z scale for each aligned pixel
    float alignedDepthOffset = 0.0f;        // (R^T T).z
};

// Per-pixel tables that convert rectified source depth to target-camera axial Z.
// R1 is the rectification rotation, R/T (T as a 1x3 row) the source-to-target extrinsics; all CV_32F.
inline DepthAlignTables depthAlignTables(
    const cv::Mat& R1, const cv::Mat& rectifiedK, cv::Size rectifiedSize, const cv::Mat& R, const cv::Mat& alignK, const cv::Mat& T, cv::Size alignSize) {
    const cv::Mat sourceZFromRectifiedPixel = R1.t() * rectifiedK.inv();
    const cv::Mat sourceZFromAlignedPixel = R.t() * alignK.inv();
    const cv::Mat sourceTranslation = R.t() * T.t();
    const auto row2 = [](const cv::Mat& m) { return std::array<float, 3>{m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2)}; };
    DepthAlignTables tables;
    tables.rectifiedRayZ = impl::imageAlignRayZ(rectifiedSize.width, rectifiedSize.height, row2(sourceZFromRectifiedPixel));
    tables.alignedDepthScales = impl::imageAlignDepthScales(alignSize.width, alignSize.height, row2(sourceZFromAlignedPixel));
    tables.alignedDepthOffset = sourceTranslation.at<float>(2);
    return tables;
}

// DepthLut is any contiguous float table with kDepthShiftLutSize entries. Callers should keep
// it off the stack: at 256 KB it overflows small thread stacks (e.g. 512 KB on macOS).
template <typename DepthLut, typename ParallelFor = ImageAlignCpuParallelFor>
inline void remapAndShiftDepth(const cv::Mat& input,
                               cv::Mat& output,
                               cv::Size rectifiedSize,
                               const std::vector<std::ptrdiff_t>& offsets,
                               const DepthLut& depthLut,
                               bool shiftPositive,
                               const std::vector<float>& rectifiedRayZ,
                               ParallelFor&& parallelFor = ParallelFor{}) {
    CV_Assert(std::size(depthLut) == kDepthShiftLutSize);
    CV_Assert(input.type() == CV_16UC1 && offsets.size() == static_cast<size_t>(rectifiedSize.area()) && rectifiedRayZ.size() == offsets.size());
    output.create(rectifiedSize, CV_16UC1);
    const int width = output.cols;
    const auto* source = input.ptr<uint16_t>();
    // Keep enough row chunks to balance work across heterogeneous CPU cores.
    parallelFor(
        cv::Range(0, output.rows),
        [&](const cv::Range& rows) {
            for(int y = rows.start; y < rows.end; ++y) {
                auto* aligned = output.ptr<uint16_t>(y);
                const auto* map = offsets.data() + static_cast<size_t>(y) * width;
                const auto* rays = rectifiedRayZ.data() + static_cast<size_t>(y) * width;
                std::fill_n(aligned, width, 0);
                // Preserve the ordered overwrite and one-pixel fill of the original shift.
                for(int x = shiftPositive ? width - 1 : 0; x >= 0 && x < width; x += shiftPositive ? -1 : 1) {
                    if(map[x] < 0) continue;
                    const uint16_t depth = source[map[x]];
                    if(depth == 0 || rays[x] == 0.0f) continue;
                    const float projectedX = x + depthLut[depth] * rays[x];
                    if(!(projectedX > -1.5f && projectedX < width + 0.5f)) continue;
                    const int u = projectedX >= 0.5f ? static_cast<int>(projectedX + 0.5f) : (projectedX <= -0.5f ? -1 : 0);
                    if(u == x) {
                        aligned[x] = depth;
                        continue;
                    }
                    for(const int target : {u, u + (shiftPositive ? 1 : -1)}) {
                        if(target >= 0 && target < width) aligned[target] = depth;
                    }
                }
            }
        },
        std::max(1.0, static_cast<double>(width) * output.rows / 16384.0));
}

template <typename ParallelFor = ImageAlignCpuParallelFor>
inline void remapAndConvertDepth(const cv::Mat& input,
                                 cv::Mat& output,
                                 const std::vector<std::ptrdiff_t>& offsets,
                                 const std::vector<float>& scales,
                                 float offset,
                                 ParallelFor&& parallelFor = ParallelFor{}) {
    CV_Assert(input.type() == CV_16UC1 && output.type() == CV_16UC1 && offsets.size() == output.total() && scales.size() == offsets.size());
    const auto* source = input.ptr<uint16_t>();
    parallelFor(
        cv::Range(0, output.rows),
        [&](const cv::Range& rows) {
            for(int y = rows.start; y < rows.end; ++y) {
                auto* target = output.ptr<uint16_t>(y);
                const size_t start = static_cast<size_t>(y) * output.cols;
                for(int x = 0; x < output.cols; ++x) {
                    const auto index = offsets[start + x];
                    target[x] = index < 0 ? 0 : impl::imageAlignScaleDepth(source[index], scales[start + x], offset);
                }
            }
        },
        std::max(1.0, static_cast<double>(output.cols) * output.rows / 16384.0));
}

}  // namespace dai::detail
