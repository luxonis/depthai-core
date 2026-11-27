#include "BleedingFilter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace dai {
namespace gate {

BleedingFilter::BleedingFilter(const Parameters& p) {
    configure(p);
}

void BleedingFilter::configure(const Parameters& p) {
    for(double value :
        {p.lambdaA, p.lambdaB, p.lambdaC, p.histogramA, p.histogramB, p.histogramC, p.sourceSigmaMm, p.nearWidthMm, p.offsetMm, p.transitionMm}) {
        if(!std::isfinite(value)) throw std::invalid_argument("ToF bleeding filter parameters must be finite");
    }
    if(p.lambdaA < 0 || p.lambdaA > 255 || p.lambdaB < 0 || p.lambdaB >= 255 || p.lambdaC <= 0 || p.histogramA < 0 || p.histogramA > 255 || p.histogramB < 0
       || p.histogramB >= 255 || p.histogramC <= 0 || p.sourceAmplitude < 1 || p.sourceAmplitude > 255 || p.protectAmplitude < 1 || p.protectAmplitude > 255
       || p.sourceSigmaMm <= 0 || p.sourceSigmaMm > maxDepthMm || p.nearWidthMm < 0 || p.nearWidthMm > maxDepthMm || p.offsetMm < 0 || p.offsetMm > maxDepthMm
       || p.transitionMm < 1 || p.transitionMm > maxDepthMm) {
        throw std::invalid_argument("ToF bleeding filter parameters out of range");
    }
    parameters = p;
    meanWeight = 0;
    for(size_t a = 0; a < weight.size(); ++a) {
        const double activation = std::clamp((a - p.lambdaB) / std::min(32.0, 255 - p.lambdaB), 0.0, 1.0);  // Ramp over 32 levels, not a jump to full strength
        lambda[a] = a < p.lambdaB ? 0 : p.lambdaA * std::pow((a - p.lambdaB) / (255 - p.lambdaB), p.lambdaC) * activation * activation * (3 - 2 * activation);
        weight[a] = a < p.histogramB ? 0 : p.histogramA * std::pow((a - p.histogramB) / (255 - p.histogramB), p.histogramC);
        meanWeight += weight[a];
    }
    meanWeight /= weight.size();
    for(size_t i = 0; i < affinity.size(); ++i) {
        const double distance = (static_cast<double>(i) - (depthBins - 1)) * binMm;
        affinity[i] = (1 + std::tanh((distance + p.nearWidthMm) / p.transitionMm)) / 2 * (1 - std::tanh((distance - p.offsetMm) / p.transitionMm)) / 2;
    }
}

void BleedingFilter::filterDepth(uint16_t* depth, const uint8_t* amplitude, size_t width, size_t height, size_t depthStride, size_t amplitudeStride) {
    if(!width || !height) return;
    if(!depth || !amplitude || depthStride < width || amplitudeStride < width || width > static_cast<size_t>(std::numeric_limits<int>::max() - 2)
       || height > static_cast<size_t>(std::numeric_limits<int>::max() - 2)) {
        throw std::invalid_argument("Invalid ToF bleeding filter image dimensions or strides");
    }
    if(meanWeight == 0 || parameters.lambdaA == 0) return;
    exterior.create(static_cast<int>(height + 2), static_cast<int>(width + 2), CV_8UC1);
    std::memset(exterior.ptr(0), 0, width + 2);
    std::memset(exterior.ptr(static_cast<int>(height + 1)), 0, width + 2);
    std::array<uint64_t, 256> histogram{};
    std::array<double, depthBins> brightDepth{};
    size_t sourceCount = 0;
    for(size_t y = 0; y < height; ++y) {
        auto* mask = exterior.ptr(static_cast<int>(y + 1));
        mask[0] = mask[width + 1] = 0;
        const auto* z = depth + y * depthStride;
        const auto* a = amplitude + y * amplitudeStride;
        for(size_t x = 0; x < width; ++x) {
            ++histogram[a[x]];
            mask[x + 1] = a[x] >= parameters.protectAmplitude;
            if(a[x] >= parameters.sourceAmplitude && z[x] > 0 && z[x] < maxDepthMm) {
                ++brightDepth[z[x] / binMm];
                ++sourceCount;
            }
        }
    }
    if(!sourceCount) return;
    double numerator = 0;
    for(size_t a = 0; a < histogram.size(); ++a) numerator += weight[a] * (static_cast<double>(histogram[a]) * a);
    const double mean = numerator / (static_cast<double>(width) * height * meanWeight);
    const double boundedMean = std::clamp(mean, 0.0, 255.0);
    const auto meanIndex = static_cast<size_t>(boundedMean);
    const double response = lambda[meanIndex] + (boundedMean - meanIndex) * (lambda[std::min(meanIndex + 1, size_t{255})] - lambda[meanIndex]);
    if(response == 0) return;

    cv::Mat source(1, static_cast<int>(depthBins), CV_64FC1, brightDepth.data());
    cv::GaussianBlur(source, smoothed, cv::Size(0, 1), parameters.sourceSigmaMm / binMm, 0, cv::BORDER_CONSTANT);
    const double* density = smoothed.ptr<double>();
    const size_t sourceBin = std::max_element(density, density + depthBins) - density;
    std::array<uint16_t, depthBins> thresholds{};
    for(size_t bin = 0; bin < depthBins - 1; ++bin) {
        thresholds[bin] = static_cast<uint16_t>(std::ceil(response * affinity[bin + depthBins - 1 - sourceBin]));
    }

    cv::floodFill(exterior, cv::Point(0, 0), cv::Scalar(2), nullptr, cv::Scalar(), cv::Scalar(), 4);  // 4-connected: dual of 8-connected bright contours
    for(size_t y = 0; y < height; ++y) {
        const auto* mask = exterior.ptr(static_cast<int>(y + 1)) + 1;
        auto* z = depth + y * depthStride;
        const auto* a = amplitude + y * amplitudeStride;
        for(size_t x = 0; x < width; ++x) {
            if(mask[x] == 2 && z[x] < maxDepthMm && a[x] < thresholds[z[x] / binMm]) z[x] = 0;
        }
    }
}

}  // namespace gate
}  // namespace dai
