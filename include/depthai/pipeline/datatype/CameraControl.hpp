#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class CameraControl : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawCameraControl& cfg;

   public:
    CameraControl();
    explicit CameraControl(std::shared_ptr<RawCameraControl> ptr);
    virtual ~CameraControl() = default;

    // Functions to set properties
    void setCaptureStill(bool capture);

    // Functions to retrieve properties
    bool getCaptureStill() const;
};

}  // namespace dai
