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

/**
 * One detected corner, as the corner detector writes it into its output buffer.
 * The low 28 bits of `score` are the Harris score (mask with HARRIS_SCORE_MASK).
 * The layout (2 + 2 + 4 bytes, no padding) mirrors dai_device_feature_point, so
 * CornerDetector::points() exposes the output buffer without a copy.
 */
struct FeaturePoint {
    uint16_t x = 0;
    uint16_t y = 0;
    uint32_t score = 0;

    /// The Harris score is in the last 28 bits of `score`
    static constexpr uint32_t HARRIS_SCORE_MASK = DAI_FEATURE_HARRIS_SCORE_MASK;
};
static_assert(sizeof(FeaturePoint) == 8, "FeaturePoint must stay a packed 8 byte record");
static_assert(sizeof(FeaturePoint) == sizeof(dai_device_feature_point), "FeaturePoint must mirror dai_device_feature_point");
static_assert(alignof(FeaturePoint) == alignof(dai_device_feature_point), "FeaturePoint must mirror dai_device_feature_point");
static_assert(offsetof(FeaturePoint, x) == offsetof(dai_device_feature_point, x), "FeaturePoint::x must mirror dai_device_feature_point::x");
static_assert(offsetof(FeaturePoint, y) == offsetof(dai_device_feature_point, y), "FeaturePoint::y must mirror dai_device_feature_point::y");
static_assert(offsetof(FeaturePoint, score) == offsetof(dai_device_feature_point, score), "FeaturePoint::score must mirror dai_device_feature_point::score");

/**
 * A feature followed over frames in the optical flow mode. Key of the node's feature map:
 * only the position takes part in the ordering, so two points at the same pixel are the same key.
 */
struct TrackedPoint {
    TrackedPoint() = default;
    TrackedPoint(uint16_t x, uint16_t y, uint32_t score, uint32_t age, uint8_t confidence) : x(x), y(y), score(score), age(age), confidence(confidence) {}

    // Only Location is used to diferentiate
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

/// Pixel position for which the descriptor calculator computes one descriptor
struct DescriptorSearchPosition {
    uint16_t x = 0;
    uint16_t y = 0;
};

/**
 * Hardware corner detector (Harris corners, 8x8 peak mode).
 * Sequence: Open(), Init(frame, runAsync), Config(); then per frame Run(frame) and read
 * points()/numPoints(); Close() releases the hardware (the destructor calls it as well).
 * The status methods return 0 on success, otherwise the status of the hardware driver.
 */
class CornerDetector {
   public:
    CornerDetector();
    ~CornerDetector();
    CornerDetector(const CornerDetector&) = delete;
    CornerDetector& operator=(const CornerDetector&) = delete;

    /// Creates the hardware session
    int Open();
    /// Initialises the detector for frames like `inFrame` (GRAY8/RAW8, at most 1920x1080, width a
    /// multiple of 128 and height a multiple of 32; throws otherwise), allocates the output buffer
    /// and starts the session
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);
    /// Applies the frame configuration set with the setters below
    int Config();
    /// Detects the corners of `inImageData` (synchronous); the result is available through points()
    void Run(std::shared_ptr<dai::ImgFrame> inImageData);
    /// Frees the output buffer and the session
    int Close();

    // Frame configuration; applied by the next Config()
    void setThreshold(uint32_t threshold);        ///< Harris score threshold (default 2000)
    void setRobustness(uint8_t robustness);       ///< default 10
    void setGradientFilterShift(uint32_t shift);  ///< default 1

    /// Corners found by the last Run(): numPoints() records, valid until the next Run() or Close()
    const FeaturePoint* points() const;
    size_t numPoints() const;

   private:
    // The detector of the vendor module. Run() is synchronous: the frame keeps its data (converted to an accelerator
    // buffer) alive while the hardware uses it.
    std::unique_ptr<dai_device_corner, void (*)(dai_device_corner*)> handle;
};

/**
 * Hardware dense optical flow between a reference frame and the current frame (bidirectional).
 * Same sequence as CornerDetector; the asynchronous mode is not supported (Init throws).
 */
class OpticalFlowAccel {
   public:
    OpticalFlowAccel();
    ~OpticalFlowAccel();
    OpticalFlowAccel(const OpticalFlowAccel&) = delete;
    OpticalFlowAccel& operator=(const OpticalFlowAccel&) = delete;

    /// Creates the hardware session
    int Open();
    /// Initialises the flow for GRAY8/RAW8 frames like `inFrame` (throws otherwise), starts the
    /// session and allocates the output maps
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);
    /// Applies the frame configuration
    int Config();
    /// Computes the flow between `refImageData` and `curImageData` (synchronous)
    void Run(std::shared_ptr<dai::ImgFrame> refImageData, std::shared_ptr<dai::ImgFrame> curImageData);
    /// Frees the output maps and the session
    int Close();

    // Results of the last Run(), one entry per pixel of the input frame (row major, width * height):
    /// Forward motion vectors (reference -> current): two int16_t per pixel, x then y; the node
    /// reads them as fixed point with 4 fractional bits
    const int16_t* forwardMotionVectors() const;
    /// Backward motion vectors (current -> reference), same layout
    const int16_t* backwardMotionVectors() const;
    /// Confidence of the motion vector of each pixel, one uint8_t per pixel
    const uint8_t* confidenceMap() const;

   private:
    dai_device_oflow_result result() const;

    // The optical flow of the vendor module. Run() is synchronous: the frames keep their data (converted to
    // accelerator buffers) alive while the hardware uses it.
    std::unique_ptr<dai_device_oflow, void (*)(dai_device_oflow*)> handle;
};

/**
 * Hardware descriptor calculator: one binary descriptor of DESCRIPTOR_SIZE bytes for each
 * search position. Same sequence as CornerDetector; the asynchronous mode is not supported
 * (Init throws).
 */
class DescriptorCalculator {
   public:
    /// Size of one descriptor in bytes
    static constexpr uint32_t DESCRIPTOR_SIZE = DAI_DESCRIPTOR_SIZE;
    /// Maximum number of search positions per Run() (the hardware buffer capacity)
    static constexpr uint32_t MAX_POINTS = DAI_DESCRIPTOR_MAX_POINTS;

    DescriptorCalculator();
    ~DescriptorCalculator();
    DescriptorCalculator(const DescriptorCalculator&) = delete;
    DescriptorCalculator& operator=(const DescriptorCalculator&) = delete;

    /// Creates the hardware session
    int Open();
    /// Initialises the calculator for GRAY8/RAW8 frames like `inFrame` (throws otherwise),
    /// allocates the output buffer and starts the session
    int Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync);
    /// Applies the frame configuration (number of points)
    int Config();
    /// Computes the descriptors of `inImageData` at `positions` (synchronous); does nothing for
    /// an empty list
    void Run(std::shared_ptr<dai::ImgFrame> inImageData, const std::vector<DescriptorSearchPosition>& positions);
    /// Frees the buffers and the session
    int Close();

    /// Number of search positions of the next Run(); applied by the next Config() (default 1000, clamped to MAX_POINTS)
    void setNumPoints(uint32_t numPoints);
    /// Whether the input is low-pass filtered before the descriptor calculation (default true)
    bool descriptorLowPassFilter() const;

    /// Descriptors of the last Run(): DESCRIPTOR_SIZE bytes per search position, in the order of `positions`
    const uint8_t* descriptors() const;

   private:
    // The descriptor calculator of the vendor module. Run() is synchronous: the frame keeps its data (converted to an
    // accelerator buffer) alive while the hardware uses it.
    std::unique_ptr<dai_device_descriptor, void (*)(dai_device_descriptor*)> handle;
    // The search positions of the last Run() in the layout of the C interface (reused to avoid an allocation per frame)
    std::vector<dai_device_point16> searchPositions;
};

}  // namespace gate
}  // namespace dai
