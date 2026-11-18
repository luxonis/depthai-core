#pragma once

#include <spdlog/spdlog.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"

namespace dai {
struct ToFConfig;
}

class tofVD55H1 {
   public:
    tofVD55H1(uint32_t rawWidth, uint32_t rawHeight, bool wantConfidence, bool wantIntensity, std::shared_ptr<spdlog::logger> logger);
    ~tofVD55H1();

    tofVD55H1(const tofVD55H1&) = delete;
    tofVD55H1& operator=(const tofVD55H1&) = delete;

    bool processFrame(const uint8_t* gray16Data, int dmaBufFd = -1);
    int64_t getNextExposureTimeNs() const;
    void applyConfig(const dai::ToFConfig& cfg);
    void buildRadialToPerpLut(const std::array<std::array<float, 3>, 3>& K, const std::vector<float>& distCoeffs);
    void buildIdentityLut();

    const std::vector<float>& radialToPerpLut() const {
        return r2pLut;
    }
    const uint8_t* getDepthPtr() const;
    size_t getDepthSizeBytes() const;
    const uint8_t* getAmplitudePtr() const;
    const uint8_t* getConfidencePtr() const;
    const uint8_t* getIntensityPtr() const;
    size_t getFloatPlaneSizeBytes() const;

    uint32_t getDepthWidth() const;
    uint32_t getDepthHeight() const;

    using Float16ToUint8Lut = std::array<uint8_t, 1 << 16>;
    static Float16ToUint8Lut makeFloat16ToUint8Log1pLut(float outScale, float inScale);
    static void transposeFloat32ToUint8(const float* src, uint8_t* dst, uint32_t srcW, uint32_t srcH, const Float16ToUint8Lut& lut);
    static void transposeFloat16ToUint8(const void* src, uint8_t* dst, uint32_t srcW, uint32_t srcH, const Float16ToUint8Lut& lut);

   private:
    const uint32_t rawWidth;
    const uint32_t rawHeight;
    const bool wantConfidence;
    const bool wantIntensity;
    uint32_t depthWidth = 0;
    uint32_t depthHeight = 0;
    size_t slotSize = 0;

    static constexpr size_t GPU_ALIGN = 128;
    struct AlignedFree {
        void operator()(void* p) const {
            std::free(p);
        }
    };
    using AlignedBuf = std::unique_ptr<uint8_t[], AlignedFree>;
    AlignedBuf alignedInput;
    AlignedBuf alignedOutput;

    std::vector<float> r2pLut;
    std::shared_ptr<spdlog::logger> logger;

    struct Impl;
    std::unique_ptr<Impl> impl;
};
