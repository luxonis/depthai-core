#include "tofVD55H1.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include <opencv2/calib3d.hpp>

// Geometry helpers and plane conversions. The sensor pipeline itself (constructor, processFrame,
// applyConfig and the amplitude/confidence/intensity planes) is implemented in the private
// accelerator module.

void tofVD55H1::buildRadialToPerpLut(const std::array<std::array<float, 3>, 3>& K, const std::vector<float>& distCoeffs) {
    const uint32_t outW = depthHeight;
    const uint32_t outH = depthWidth;

    std::vector<cv::Point2f> pts;
    pts.reserve(static_cast<size_t>(outW) * outH);
    for(uint32_t v = 0; v < outH; ++v) {
        for(uint32_t u = 0; u < outW; ++u) {
            pts.emplace_back(static_cast<float>(u), static_cast<float>(v));
        }
    }

    const cv::Matx33d cvK(K[0][0], K[0][1], K[0][2], K[1][0], K[1][1], K[1][2], K[2][0], K[2][1], K[2][2]);
    const std::vector<double> cvDist(distCoeffs.begin(), distCoeffs.end());
    std::vector<cv::Point2f> undist;
    cv::undistortPoints(pts, undist, cvK, cvDist);

    r2pLut.resize(static_cast<size_t>(outW) * outH);
    for(size_t i = 0; i < undist.size(); ++i) {
        const float xn = undist[i].x;
        const float yn = undist[i].y;
        r2pLut[i] = 1.0f / std::sqrt(1.0f + xn * xn + yn * yn);
    }
}

void tofVD55H1::buildIdentityLut() {
    // No calibration: skip radial-to-perpendicular correction (factor 1.0 per pixel).
    r2pLut.assign(static_cast<size_t>(depthWidth) * depthHeight, 1.0f);
}

const uint8_t* tofVD55H1::getDepthPtr() const {
    return alignedOutput.get();
}

size_t tofVD55H1::getDepthSizeBytes() const {
    return static_cast<size_t>(depthWidth) * depthHeight * sizeof(uint16_t);
}

size_t tofVD55H1::getFloatPlaneSizeBytes() const {
    return slotSize;
}

uint32_t tofVD55H1::getDepthWidth() const {
    return depthWidth;
}

uint32_t tofVD55H1::getDepthHeight() const {
    return depthHeight;
}

tofVD55H1::Float16ToUint8Lut tofVD55H1::makeFloat16ToUint8Log1pLut(float outScale, float inScale) {
    Float16ToUint8Lut lut;
    for(uint32_t bits = 0; bits < lut.size(); ++bits) {
        __fp16 value;
        static_assert(sizeof(value) == sizeof(uint16_t));
        const auto halfBits = static_cast<uint16_t>(bits);
        std::memcpy(&value, &halfBits, sizeof(value));
        const float mapped = outScale * std::log1p(inScale * static_cast<float>(value));
        lut[bits] = !std::isfinite(mapped) ? 0 : static_cast<uint8_t>(mapped < 0.0f ? 0.0f : (mapped > 255.0f ? 255.0f : mapped) + 0.5f);
    }
    return lut;
}

void tofVD55H1::transposeFloat32ToUint8(const float* src, uint8_t* dst, uint32_t srcW, uint32_t srcH, const Float16ToUint8Lut& lut) {
    for(uint32_t srcY = 0; srcY < srcH; ++srcY) {
        for(uint32_t srcX = 0; srcX < srcW; ++srcX) {
            const __fp16 value = static_cast<__fp16>(src[srcX + srcY * srcW]);
            uint16_t bits;
            std::memcpy(&bits, &value, sizeof(bits));
            dst[srcY + srcX * srcH] = lut[bits];
        }
    }
}

void tofVD55H1::transposeFloat16ToUint8(const void* src, uint8_t* dst, uint32_t srcW, uint32_t srcH, const Float16ToUint8Lut& lut) {
    const auto* in = static_cast<const uint16_t*>(src);
    for(uint32_t srcY = 0; srcY < srcH; ++srcY) {
        for(uint32_t srcX = 0; srcX < srcW; ++srcX) {
            dst[srcY + srcX * srcH] = lut[in[srcX + srcY * srcW]];
        }
    }
}
