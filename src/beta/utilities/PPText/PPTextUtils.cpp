#include "beta/utilities/PPText/PPTextUtils.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

#include "utility/ErrorMacros.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <algorithm>
    #include <cmath>
    #include <cstddef>
    #include <cstring>
    #include <numeric>
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
    #include <utility>
    #include <vector>
#endif

namespace dai {
namespace beta {
namespace utilities {
namespace PPTextUtils {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

constexpr double PI = 3.14159265358979323846;

std::string formatShape(const std::vector<std::size_t>& dims) {
    std::string result = "(";
    for(std::size_t i = 0; i < dims.size(); i++) {
        if(i > 0) {
            result += ", ";
        }
        result += std::to_string(dims[i]);
    }
    if(dims.size() == 1) {
        result += ",";
    }
    result += ")";
    return result;
}

/*
 * When more contours than max_detections survive the area filter, the source selects the
 * max_detections largest by area with np.argpartition(-contour_areas, max_detections)
 * [:max_detections] and processes them in the resulting (arbitrary but deterministic) order.
 * The helpers below replicate that order with a C++ port of NumPy's generic (portable scalar)
 * argpartition (introselect) algorithm on double values, taken from
 * numpy/_core/src/npysort/selection.cpp and numpy/_core/src/common/numpy_tag.h at tag v2.5.1,
 * structured like the float port in src/beta/utilities/MLSD/MLSDUtils.cpp.
 *
 * NumPy is licensed under the BSD 3-Clause License:
 *   Copyright (c) 2005-2025, NumPy Developers. All rights reserved.
 * The quickselect is loosely based on the public domain implementation by Nicolas Devillard
 * (http://ndevilla.free.fr/median/median/). See
 * https://github.com/numpy/numpy/blob/v2.5.1/LICENSE.txt for the full license text;
 * redistribution of this derived code retains the above copyright notice per the license.
 *
 * Note: recent numpy builds dispatch argpartition of 64-bit dtypes to CPU-specific
 * x86-simd-sort kernels on AVX-512 hardware, whose ordering of equal values differs per CPU.
 * The generic scalar algorithm ported here is numpy's portable semantics used on all other
 * hardware.
 */

inline bool doubleLess(double a, double b) {  // As numpy floating_point_type::less: NaN sorts to the end
    return a < b || (b != b && a == a);
}

inline int npyGetMsb(std::int64_t num) {
    int msb = 0;
    while(num >>= 1) {
        msb++;
    }
    return msb;
}

std::int64_t argIntroSelect(const double* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth);

void argMedian3Swap(const double* v, std::int64_t* tosort, std::int64_t low, std::int64_t mid, std::int64_t high) {
    if(doubleLess(v[tosort[high]], v[tosort[mid]])) {
        std::swap(tosort[high], tosort[mid]);
    }
    if(doubleLess(v[tosort[high]], v[tosort[low]])) {
        std::swap(tosort[high], tosort[low]);
    }
    if(doubleLess(v[tosort[low]], v[tosort[mid]])) {
        std::swap(tosort[low], tosort[mid]);
    }
    std::swap(tosort[mid], tosort[low + 1]);
}

std::int64_t argMedian5(const double* v, std::int64_t* tosort) {
    if(doubleLess(v[tosort[1]], v[tosort[0]])) {
        std::swap(tosort[1], tosort[0]);
    }
    if(doubleLess(v[tosort[4]], v[tosort[3]])) {
        std::swap(tosort[4], tosort[3]);
    }
    if(doubleLess(v[tosort[3]], v[tosort[0]])) {
        std::swap(tosort[3], tosort[0]);
    }
    if(doubleLess(v[tosort[4]], v[tosort[1]])) {
        std::swap(tosort[4], tosort[1]);
    }
    if(doubleLess(v[tosort[2]], v[tosort[1]])) {
        std::swap(tosort[2], tosort[1]);
    }
    if(doubleLess(v[tosort[3]], v[tosort[2]])) {
        if(doubleLess(v[tosort[3]], v[tosort[1]])) {
            return 1;
        } else {
            return 3;
        }
    } else {
        return 2;
    }
}

std::int64_t argMedianOfMedian5(const double* v, std::int64_t* tosort, std::int64_t num) {
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

void argDumbSelect(const double* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth) {
    for(std::int64_t i = 0; i <= kth; i++) {
        std::int64_t minidx = i;
        double minval = v[tosort[i]];
        for(std::int64_t k = i + 1; k < num; k++) {
            if(doubleLess(v[tosort[k]], minval)) {
                minidx = k;
                minval = v[tosort[k]];
            }
        }
        std::swap(tosort[i], tosort[minidx]);
    }
}

void argUnguardedPartition(const double* v, std::int64_t* tosort, const double pivot, std::int64_t* ll, std::int64_t* hh) {
    for(;;) {
        do {
            (*ll)++;
        } while(doubleLess(v[tosort[*ll]], pivot));
        do {
            (*hh)--;
        } while(doubleLess(pivot, v[tosort[*hh]]));

        if(*hh < *ll) {
            break;
        }

        std::swap(tosort[*ll], tosort[*hh]);
    }
}

std::int64_t argIntroSelect(const double* v, std::int64_t* tosort, std::int64_t num, std::int64_t kth) {  // numpy introselect_ without the unused pivot stack
    std::int64_t low = 0;
    std::int64_t high = num - 1;

    if(kth - low < 3) {
        argDumbSelect(v, tosort + low, high - low + 1, kth - low);
        return 0;
    } else if(kth == num - 1) {
        std::int64_t maxidx = low;
        double maxval = v[tosort[low]];
        for(std::int64_t k = low + 1; k < num; k++) {
            if(!doubleLess(v[tosort[k]], maxval)) {
                maxidx = k;
                maxval = v[tosort[k]];
            }
        }
        std::swap(tosort[kth], tosort[maxidx]);
        return 0;
    }

    int depthLimit = npyGetMsb(num) * 2;

    for(; low + 1 < high;) {
        std::int64_t ll = low + 1;
        std::int64_t hh = high;

        if(depthLimit > 0 || hh - ll < 5) {  // Median-of-median5 fallback for a linear worst case; small sizes need med3 for the unguarded partition
            const std::int64_t mid = low + (high - low) / 2;
            argMedian3Swap(v, tosort, low, mid, high);
        } else {
            const std::int64_t mid = ll + argMedianOfMedian5(v, tosort + ll, hh - ll);
            std::swap(tosort[mid], tosort[low]);
            ll--;  // Adapt for the larger partition than the med3 pivot
            hh++;
        }

        depthLimit--;

        argUnguardedPartition(v, tosort, v[tosort[low]], &ll, &hh);  // The previous swapping removes the need for bound checks

        std::swap(tosort[low], tosort[hh]);

        if(hh >= kth) {
            high = hh - 1;
        }
        if(hh <= kth) {
            low = ll;
        }
    }

    if(high == low + 1) {
        if(doubleLess(v[tosort[high]], v[tosort[low]])) {
            std::swap(tosort[high], tosort[low]);
        }
    }

    return 0;
}

constexpr std::ptrdiff_t PW_BLOCKSIZE = 128;  // Port of the NumPy (BSD-3-Clause) pairwise sum, numpy/_core/src/umath/loops_utils.h.src v2.5.1

float pairwiseSum(const float* a, std::ptrdiff_t n) {
    if(n < 8) {
        float res = -0.0f;  // -0.0f keeps a sum of only -0 values at -0, because 0 + -0 == 0
        for(std::ptrdiff_t i = 0; i < n; i++) {
            res += a[i];
        }
        return res;
    } else if(n <= PW_BLOCKSIZE) {
        float r[8];
        for(int j = 0; j < 8; j++) {
            r[j] = a[j];
        }
        std::ptrdiff_t i = 8;
        for(; i < n - (n % 8); i += 8) {
            for(int j = 0; j < 8; j++) {
                r[j] += a[i + j];
            }
        }
        float res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for(; i < n; i++) {
            res += a[i];
        }
        return res;
    } else {
        std::ptrdiff_t n2 = n / 2;
        n2 -= n2 % 8;
        return pairwiseSum(a, n2) + pairwiseSum(a + n2, n - n2);
    }
}

std::array<float, 5> cornersToRotatedBbox(const std::array<cv::Point2f, 4>& corners) {  // Returns [xCenter, yCenter, width, height, angle]
    if(corners[0].x >= corners[1].x) {
        throw std::runtime_error("Corners must be ordered by top-left, top-right, bottom-right, bottom-left. Got top-right corner before top-left.");
    }
    if(corners[0].y >= corners[3].y) {
        throw std::runtime_error("Corners must be ordered by top-left, top-right, bottom-right, bottom-left. Got bottom-left corner before top-left.");
    }
    if(corners[1].y >= corners[2].y) {
        throw std::runtime_error("Corners must be ordered by top-left, top-right, bottom-right, bottom-left. Got bottom-right corner before top-right.");
    }
    if(corners[3].x >= corners[2].x) {
        throw std::runtime_error("Corners must be ordered by top-left, top-right, bottom-right, bottom-left. Got bottom-left corner before bottom-right.");
    }

    const float xDist = corners[1].x - corners[0].x;
    const float yDist = corners[1].y - corners[0].y;

    const float angle = std::atan2(yDist, xDist) * static_cast<float>(180.0 / PI);

    float xSum = corners[0].x + corners[1].x;
    xSum = xSum + corners[2].x;
    xSum = xSum + corners[3].x;
    float ySum = corners[0].y + corners[1].y;
    ySum = ySum + corners[2].y;
    ySum = ySum + corners[3].y;
    const float xCenter = xSum / 4.0f;
    const float yCenter = ySum / 4.0f;

    const float widthDx = corners[0].x - corners[1].x;  // Separate statements keep the mul/add rounding unfused, as np.linalg.norm
    const float widthDy = corners[0].y - corners[1].y;
    const float widthDx2 = widthDx * widthDx;
    const float widthDy2 = widthDy * widthDy;
    const float width = std::sqrt(widthDx2 + widthDy2);
    const float heightDx = corners[1].x - corners[2].x;
    const float heightDy = corners[1].y - corners[2].y;
    const float heightDx2 = heightDx * heightDx;
    const float heightDy2 = heightDy * heightDy;
    const float height = std::sqrt(heightDx2 + heightDy2);

    return {xCenter, yCenter, width, height, angle};
}

struct MiniBox {
    std::array<float, 5> bbox;
    std::array<cv::Point2f, 4> corners;  // Ordered top-left, top-right, bottom-right, bottom-left
};

MiniBox getMiniBoxes(const std::vector<cv::Point>& contour) {
    const cv::RotatedRect boundingBox = cv::minAreaRect(contour);
    std::array<cv::Point2f, 4> points;
    boundingBox.points(points.data());  // cv2.boxPoints
    std::stable_sort(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

    int index1 = 0;
    int index2 = 1;
    int index3 = 2;
    int index4 = 3;
    if(points[1].y > points[0].y) {
        index1 = 0;
        index4 = 1;
    } else {
        index1 = 1;
        index4 = 0;
    }
    if(points[3].y > points[2].y) {
        index2 = 2;
        index3 = 3;
    } else {
        index2 = 3;
        index3 = 2;
    }

    const std::array<cv::Point2f, 4> corners{points[index1], points[index2], points[index3], points[index4]};
    return MiniBox{cornersToRotatedBbox(corners), corners};
}

void dilateBox(std::array<cv::Point2f, 4>& corners, int pixels) {
    const float offset = static_cast<float>(pixels);
    corners[0].x -= offset;
    corners[0].y -= offset;
    corners[1].x += offset;
    corners[1].y -= offset;
    corners[2].x += offset;
    corners[2].y += offset;
    corners[3].x -= offset;
    corners[3].y += offset;
}

double boxScore(const cv::Mat& predictions, std::array<cv::Point2f, 4> corners) {
    const int height = predictions.rows;
    const int width = predictions.cols;
    dilateBox(corners, -2);

    cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
    std::vector<std::vector<cv::Point>> polygon(1);
    polygon[0].reserve(corners.size());
    for(const auto& corner : corners) {
        polygon[0].emplace_back(static_cast<int>(corner.x), static_cast<int>(corner.y));
    }
    cv::fillPoly(mask, polygon, cv::Scalar(1));

    std::vector<float> masked(static_cast<std::size_t>(height) * static_cast<std::size_t>(width));
    std::uint64_t maskSum = 0;
    for(int row = 0; row < height; row++) {
        const float* predictionsRow = predictions.ptr<float>(row);
        const std::uint8_t* maskRow = mask.ptr<std::uint8_t>(row);
        float* maskedRow = masked.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(width);
        for(int col = 0; col < width; col++) {
            maskedRow[col] = predictionsRow[col] * static_cast<float>(maskRow[col]);
            maskSum += maskRow[col];
        }
    }
    const float numerator = 0.0f + pairwiseSum(masked.data(), static_cast<std::ptrdiff_t>(masked.size()));  // np.sum starts from the additive identity 0

    return static_cast<double>(numerator) / (static_cast<double>(maskSum) + 1e-5);
}

}  // namespace

PPTextDetections parsePaddleDetectionOutputs(
    const std::vector<float>& predictions, const std::vector<std::size_t>& dims, float maskThreshold, float bboxThreshold, int maxDetections) {
    DAI_CHECK_V(dims.size() == 4, "Predictions should be 4D array of shape (1, 1, H, W) or (1, H, W, 1), got {}.", formatShape(dims));
    std::size_t height = 0;
    std::size_t width = 0;
    if(dims[0] == 1 && dims[1] == 1) {
        height = dims[2];
        width = dims[3];
    } else if(dims[0] == 1 && dims[3] == 1) {
        height = dims[1];
        width = dims[2];
    } else {
        DAI_CHECK_V(false, "Predictions should be either (1, 1, H, W) or (1, H, W, 1), got {}.", formatShape(dims));
    }
    DAI_CHECK_V(
        predictions.size() == height * width, "Cannot interpret {} prediction values as a probability map of shape {}.", predictions.size(), formatShape(dims));

    const cv::Mat predictionsMat(static_cast<int>(height), static_cast<int>(width), CV_32F, const_cast<float*>(predictions.data()));

    cv::Mat mask(static_cast<int>(height), static_cast<int>(width), CV_8U);
    for(int row = 0; row < static_cast<int>(height); row++) {
        const float* predictionsRow = predictionsMat.ptr<float>(row);
        std::uint8_t* maskRow = mask.ptr<std::uint8_t>(row);
        for(int col = 0; col < static_cast<int>(width); col++) {
            maskRow[col] = predictionsRow[col] > maskThreshold ? 1 : 0;
        }
    }
    const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::dilate(mask, mask, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> filtered;
    filtered.reserve(contours.size());
    for(auto& contour : contours) {
        if(cv::contourArea(contour) > 8.0) {
            filtered.push_back(std::move(contour));
        }
    }

    const std::int64_t num = static_cast<std::int64_t>(filtered.size());
    if(num > static_cast<std::int64_t>(maxDetections)) {
        std::vector<double> negAreas(num);
        for(std::int64_t i = 0; i < num; i++) {
            negAreas[i] = -cv::contourArea(filtered[i]);
        }
        std::int64_t kth = maxDetections;
        if(kth < 0) {
            kth += num;
        }
        DAI_CHECK_V(kth >= 0 && kth < num, "kth(={}) out of bounds ({}).", maxDetections, num);
        std::vector<std::int64_t> tosort(num);
        std::iota(tosort.begin(), tosort.end(), 0);
        argIntroSelect(negAreas.data(), tosort.data(), num, kth);

        const std::int64_t sliceEnd = maxDetections >= 0 ? static_cast<std::int64_t>(maxDetections) : std::max<std::int64_t>(0, num + maxDetections);
        std::vector<std::vector<cv::Point>> selected;
        selected.reserve(sliceEnd);
        for(std::int64_t i = 0; i < sliceEnd; i++) {
            selected.push_back(std::move(filtered[tosort[i]]));
        }
        filtered = std::move(selected);
    }

    PPTextDetections result;
    for(const auto& contour : filtered) {
        const MiniBox miniBox = getMiniBoxes(contour);
        if(std::min(miniBox.bbox[2], miniBox.bbox[3]) < 8.0f) {  // Skip tiny boxes
            continue;
        }
        const double score = boxScore(predictionsMat, miniBox.corners);
        if(score < static_cast<double>(bboxThreshold)) {
            continue;
        }
        const float unclippedWidth = static_cast<float>(static_cast<double>(miniBox.bbox[2]) * std::sqrt(2.0));
        const float unclippedHeight = static_cast<float>(static_cast<double>(miniBox.bbox[3]) * std::sqrt(2.0));

        result.bboxes.push_back({miniBox.bbox[0] / static_cast<float>(width),
                                 miniBox.bbox[1] / static_cast<float>(height),
                                 unclippedWidth / static_cast<float>(width),
                                 unclippedHeight / static_cast<float>(height)});
        result.angles.push_back(std::rint(miniBox.bbox[4]));  // Half to even, as round(angle, 0) in the source
        result.scores.push_back(static_cast<float>(score));
    }

    for(auto& bbox : result.bboxes) {
        for(auto& value : bbox) {
            value = std::min(std::max(value, 0.0f), 1.0f);
        }
    }
    return result;
}

#else

PPTextDetections parsePaddleDetectionOutputs(
    const std::vector<float>& predictions, const std::vector<std::size_t>& dims, float maskThreshold, float bboxThreshold, int maxDetections) {
    (void)predictions;
    (void)dims;
    (void)maskThreshold;
    (void)bboxThreshold;
    (void)maxDetections;
    throw std::runtime_error("PPTextDetectionParser requires OpenCV support.");
}

#endif

}  // namespace PPTextUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
