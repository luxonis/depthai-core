#pragma once
#include <cstdint>

#include "depthai/properties/Properties.hpp"
namespace dai {
/** Initial configuration for causal, motion-aware RVC4 deblurring. */
struct DeblurProperties : PropertiesSerializable<Properties, DeblurProperties> {
    /// Restoration blend in [0,1].
    float strength = 0.7f;
    /// Positive Wiener regularization in [0.001,1].
    float regularization = 0.04f;
    /// Pre-truncation Wiener spectral gain limit in [1,8]; the final FIR can exceed it.
    float maxGain = 3.0f;
    /// Minimum exposure-scaled blur length, in pixels.
    float minBlur = 0.8f;
    /// Maximum blur length, in pixels, at most 24.
    float maxBlur = 12.0f;
    /// Motion vector quantization step in pixels, at least 0.25.
    float motionQuantum = 0.5f;
    /// Base forward/backward consistency tolerance in pixels.
    float forwardBackwardThreshold = 1.5f;
    /// Maximum smoothed correspondence residual in 8-bit intensity units.
    float photometricThreshold = 25.0f;
    /// Minimum local variance in squared 8-bit intensity units.
    float minTextureVariance = 9.0f;
    /// Raw EVA confidence cutoff [0,255], sampled on reference grid by tracker convention.
    float minConfidence = 128.0f;
    /// Maximum absolute per-pixel correction in 8-bit intensity units [0,255].
    float maxCorrection = 24.0f;
    /// FIR radius in [1,12]; full kernel width is 2*radius+1.
    std::uint32_t kernelRadius = 6;
    /// Maximum number of quantized motion kernels [1,128].
    std::uint32_t maxClusters = 32;
    /// Exposure override in microseconds; zero uses target frame exposure metadata.
    std::uint32_t exposureTimeUs = 0;
    /// Maximum wait for matching flow, in milliseconds [0,1000], excluding processing/backpressure.
    std::uint32_t maxWaitMs = 100;
    /// Bound on retained image and flow entries [2,32].
    std::uint32_t cacheSize = 8;
    /// Run FIR on the RVC4 GPU. False selects CPU reference. GPU failures stop this node; no silent fallback.
    bool useOpenCL = true;
    ~DeblurProperties() override;
};
DEPTHAI_SERIALIZE_EXT_DECL(DeblurProperties,
                           strength,
                           regularization,
                           maxGain,
                           minBlur,
                           maxBlur,
                           motionQuantum,
                           forwardBackwardThreshold,
                           photometricThreshold,
                           minTextureVariance,
                           minConfidence,
                           maxCorrection,
                           kernelRadius,
                           maxClusters,
                           exposureTimeUs,
                           maxWaitMs,
                           cacheSize,
                           useOpenCL);
}  // namespace dai
extern template struct dai::PropertiesSerializable<dai::Properties, dai::DeblurProperties>;
