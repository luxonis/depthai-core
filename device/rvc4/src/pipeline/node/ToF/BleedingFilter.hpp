#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace dai {
namespace gate {

// One instance per ToF node; configure/process on that node's thread.
class BleedingFilter {
   public:
    struct Parameters {
        // Amplitude-only calibration; activation and decisions are stabilized below.
        double lambdaA = 104;
        double lambdaB = 28;
        double lambdaC = 0.022351650527448755;
        double histogramA = 1;
        double histogramB = 144;
        double histogramC = 2.2134913537941525;
        int sourceAmplitude = 184;
        int protectAmplitude = 128;
        double sourceSigmaMm = 75;
        double nearWidthMm = 1375;
        double offsetMm = 875;
        double transitionMm = 50;
    };

    explicit BleedingFilter(const Parameters& parameters);
    void configure(const Parameters& parameters);

    // Depth in mm, amplitude in uint8; strides in elements. Zeros and >=10 m stay unchanged.
    // Scratch storage is reused, so this method is not reentrant.
    void filterDepth(uint16_t* depth, const uint8_t* amplitude, size_t width, size_t height, size_t depthStride, size_t amplitudeStride);

   private:
    static constexpr size_t binMm = 25;
    static constexpr size_t maxDepthMm = 10000;
    static constexpr size_t depthBins = maxDepthMm / binMm + 1;
    Parameters parameters;
    std::array<double, 256> lambda{};
    std::array<double, 256> weight{};
    std::array<double, 2 * depthBins - 1> affinity{};
    double meanWeight = 0;
    cv::Mat exterior;
    cv::Mat smoothed;
};

}  // namespace gate
}  // namespace dai
