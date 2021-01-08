#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace dai {

std::shared_ptr<RawBuffer> CameraControl::serialize() const {
    return raw;
}

CameraControl::CameraControl() : Buffer(std::make_shared<RawCameraControl>()), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}
CameraControl::CameraControl(std::shared_ptr<RawCameraControl> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}

// helpers
// Functions to set properties
void CameraControl::setCaptureStill(bool capture) {
    // Enable capture
    cfg.captureStill = capture;
}

bool CameraControl::getCaptureStill() const {
    return cfg.captureStill;
}

}  // namespace dai
