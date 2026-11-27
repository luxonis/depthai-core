#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "vendor/dai_device_accel.h"

namespace dai {
namespace gate {

struct FeaturePoint {
    uint16_t x = 0;
    uint16_t y = 0;
    uint32_t score = 0;

    static constexpr uint32_t HARRIS_SCORE_MASK = DAI_FEATURE_HARRIS_SCORE_MASK;  // The Harris score is in the low 28 bits of score
};
static_assert(sizeof(FeaturePoint) == 8, "FeaturePoint must stay a packed 8 byte record");
static_assert(sizeof(FeaturePoint) == sizeof(dai_device_feature_point), "FeaturePoint must mirror dai_device_feature_point");
static_assert(alignof(FeaturePoint) == alignof(dai_device_feature_point), "FeaturePoint must mirror dai_device_feature_point");
static_assert(offsetof(FeaturePoint, x) == offsetof(dai_device_feature_point, x), "FeaturePoint::x must mirror dai_device_feature_point::x");
static_assert(offsetof(FeaturePoint, y) == offsetof(dai_device_feature_point, y), "FeaturePoint::y must mirror dai_device_feature_point::y");
static_assert(offsetof(FeaturePoint, score) == offsetof(dai_device_feature_point, score), "FeaturePoint::score must mirror dai_device_feature_point::score");

struct TrackedPoint {
    TrackedPoint() = default;
    TrackedPoint(uint16_t x, uint16_t y, uint32_t score, uint32_t age, uint8_t confidence) : x(x), y(y), score(score), age(age), confidence(confidence) {}

    bool operator<(const TrackedPoint& other) const {
        return (y < other.y) || ((y == other.y) && (x < other.x));
    }

    float distance(const TrackedPoint& other) const {
        return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
    }

    uint16_t x = 0;
    uint16_t y = 0;
    uint32_t score = 0;
    uint32_t age = 0;
    uint8_t confidence = 0;
};

struct DescriptorSearchPosition {
    uint16_t x = 0;
    uint16_t y = 0;
};

class CornerDetector {  // Call Open(), Init(), Config(), then Run() per frame; status methods return 0 or the driver status
   public:
    CornerDetector();
    ~CornerDetector();
    CornerDetector(const CornerDetector&) = delete;
    CornerDetector& operator=(const CornerDetector&) = delete;

    int Open();
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);  // inFrame: GRAY8/RAW8, max 1920x1080, width % 128 == 0, height % 32 == 0; else throws
    int Config();                                                     // Applies the frame configuration set with the setters below
    void Run(std::shared_ptr<dai::ImgFrame> inImageData);
    int Close();

    void setThreshold(uint32_t threshold);        ///< Harris score threshold (default 2000)
    void setRobustness(uint8_t robustness);       ///< default 10
    void setGradientFilterShift(uint32_t shift);  ///< default 1

    const FeaturePoint* points() const;  // numPoints() records of the last Run(), valid until the next Run() or Close()
    size_t numPoints() const;

   private:
    std::unique_ptr<dai_device_corner, void (*)(dai_device_corner*)> handle;
};

class OpticalFlow {
   public:
    OpticalFlow();
    ~OpticalFlow();
    OpticalFlow(const OpticalFlow&) = delete;
    OpticalFlow& operator=(const OpticalFlow&) = delete;

    int Open();
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);  // inFrame: GRAY8/RAW8, else throws
    int Config();
    void Run(std::shared_ptr<dai::ImgFrame> refImageData, std::shared_ptr<dai::ImgFrame> curImageData);
    int Close();

    const int16_t* forwardMotionVectors() const;  // Last Run(), row major per pixel: x, y int16 pairs, fixed point with 4 fractional bits
    const int16_t* backwardMotionVectors() const;
    const uint8_t* confidenceMap() const;

   private:
    dai_device_oflow_result result() const;

    std::unique_ptr<dai_device_oflow, void (*)(dai_device_oflow*)> handle;
};

class DescriptorCalculator {
   public:
    static constexpr uint32_t DESCRIPTOR_SIZE = DAI_DESCRIPTOR_SIZE;
    static constexpr uint32_t MAX_POINTS = DAI_DESCRIPTOR_MAX_POINTS;

    DescriptorCalculator();
    ~DescriptorCalculator();
    DescriptorCalculator(const DescriptorCalculator&) = delete;
    DescriptorCalculator& operator=(const DescriptorCalculator&) = delete;

    int Open();
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);  // inFrame: GRAY8/RAW8, else throws
    int Config();
    void Run(std::shared_ptr<dai::ImgFrame> inImageData, const std::vector<DescriptorSearchPosition>& positions);
    int Close();

    void setNumPoints(uint32_t numPoints);  // Applied by the next Config(); default 1000, clamped to MAX_POINTS
    bool descriptorLowPassFilter() const;

    const uint8_t* descriptors() const;  // Last Run(): DESCRIPTOR_SIZE bytes per search position, in the order of positions

   private:
    std::unique_ptr<dai_device_descriptor, void (*)(dai_device_descriptor*)> handle;
    std::vector<dai_device_point16> searchPositions;
};

}  // namespace gate
}  // namespace dai
