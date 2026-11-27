#include "tofVD55H1.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <string>

#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "utilities/ErrorMacros.hpp"
#include "vendor/DeviceError.hpp"

namespace {

constexpr const char* UNIT = "ToF";

uint32_t toDeviceProfile(dai::ToFConfig::Profile profile) {
    switch(profile) {
        case dai::ToFConfig::Profile::LOW_RANGE:
            return DAI_TOF_PROFILE_LOW_RANGE;
        case dai::ToFConfig::Profile::MID_RANGE:
            return DAI_TOF_PROFILE_MID_RANGE;
        case dai::ToFConfig::Profile::HIGH_RANGE:
            return DAI_TOF_PROFILE_HIGH_RANGE;
    }
    throw std::runtime_error("tofVD55H1: unknown ToF profile " + std::to_string(static_cast<std::uint32_t>(profile)));
}

}  // namespace

tofVD55H1::tofVD55H1(uint32_t rawWidth, uint32_t rawHeight, bool wantConfidence, bool wantIntensity, std::shared_ptr<spdlog::logger> logger)
    : handle(nullptr, &dai_device_tof_destroy), logger(std::move(logger)) {
    dai::device::installDeviceLog();

    dai_device_tof_params params{};
    params.raw_width = rawWidth;
    params.raw_height = rawHeight;
    params.want_confidence = wantConfidence ? 1 : 0;
    params.want_intensity = wantIntensity ? 1 : 0;

    dai_device_tof* tof = nullptr;
    const dai_status status = dai_device_tof_create(&params, &tof);
    if(status != DAI_STATUS_OK) {
#ifdef DEPTHAI_DEVICE_TOF_VD55H1_STUB
        throw dai::device::DeviceError(
            status, "dai_device_tof_create", "tofVD55H1: ToF VD55H1 support is not built into this firmware (DEPTHAI_DEVICE_TOF_VD55H1=OFF)");
#else
        dai::device::throwDeviceError(status, UNIT, "dai_device_tof_create", nullptr);
#endif
    }
    handle.reset(tof);
}

tofVD55H1::~tofVD55H1() = default;

bool tofVD55H1::processFrame(const uint8_t* gray16Data, int dmaBufFd) {
    const dai_status status = dai_device_tof_process(handle.get(), gray16Data, dmaBufFd);
    if(status != DAI_STATUS_OK) {
        if(logger) {
            logger->error("[tofVD55H1] processFrame failed: {}: {}", dai::device::describeStatus(status), dai_device_last_error());
        }
        return false;
    }
    return true;
}

int64_t tofVD55H1::getNextExposureTimeNs() const {
    return dai_device_tof_next_exposure_ns(handle.get());
}

void tofVD55H1::applyConfig(const dai::ToFConfig& cfg) {
    dai_device_tof_config config{};
    config.profile = toDeviceProfile(cfg.profile);
    config.phase_shuffle_temporal_filter = cfg.enablePhaseShuffleTemporalFilter ? 1 : 0;
    DAI_CHECK_DEVICE(UNIT, dai_device_tof_set_config(handle.get(), &config));
}

dai_device_tof_output tofVD55H1::output() const {
    dai_device_tof_output out{};
    DAI_CHECK_DEVICE(UNIT, dai_device_tof_output_get(handle.get(), &out));
    return out;
}

void tofVD55H1::buildRadialToPerpLut(const std::array<std::array<float, 3>, 3>& K, const std::vector<float>& distCoeffs) {
    const auto sizes = output();
    const uint32_t outW = sizes.depth_height;
    const uint32_t outH = sizes.depth_width;

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
    const auto sizes = output();
    r2pLut.assign(static_cast<size_t>(sizes.depth_width) * sizes.depth_height, 1.0f);
}

const uint8_t* tofVD55H1::getDepthPtr() const {
    return output().depth;
}

size_t tofVD55H1::getDepthSizeBytes() const {
    return output().depth_size;
}

const uint8_t* tofVD55H1::getAmplitudePtr() const {
    return output().amplitude;
}

const uint8_t* tofVD55H1::getConfidencePtr() const {
    return output().confidence;
}

const uint8_t* tofVD55H1::getIntensityPtr() const {
    return output().intensity;
}

size_t tofVD55H1::getFloatPlaneSizeBytes() const {
    return output().float_plane_size;
}

uint32_t tofVD55H1::getDepthWidth() const {
    return output().depth_width;
}

uint32_t tofVD55H1::getDepthHeight() const {
    return output().depth_height;
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
