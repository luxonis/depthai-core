#include "pipeline/node/ToF/tofVD55H1.hpp"

#include <stdexcept>

struct tofVD55H1::Impl {};

tofVD55H1::tofVD55H1(uint32_t rawWidth, uint32_t rawHeight, bool wantConfidence, bool wantIntensity, std::shared_ptr<spdlog::logger> logger)
    : rawWidth(rawWidth), rawHeight(rawHeight), wantConfidence(wantConfidence), wantIntensity(wantIntensity), logger(std::move(logger)) {
    throw std::runtime_error("tofVD55H1: ToF VD55H1 support is not built into this firmware (DEPTHAI_DEVICE_TOF_VD55H1=OFF)");
}

tofVD55H1::~tofVD55H1() = default;

bool tofVD55H1::processFrame(const uint8_t*, int) {
    return false;
}

int64_t tofVD55H1::getNextExposureTimeNs() const {
    return 0;
}

void tofVD55H1::applyConfig(const dai::ToFConfig&) {}

const uint8_t* tofVD55H1::getAmplitudePtr() const {
    return nullptr;
}

const uint8_t* tofVD55H1::getConfidencePtr() const {
    return nullptr;
}

const uint8_t* tofVD55H1::getIntensityPtr() const {
    return nullptr;
}
