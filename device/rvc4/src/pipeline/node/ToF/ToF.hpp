#pragma once

#include <depthai/properties/ToFProperties.hpp>
#include <memory>
#include <string>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "tofVD55H1.hpp"

namespace dai {
class ImgFrame;

namespace gate {

class Camera;

class ToFBase : public dai::node::ToFBase {
   public:
    constexpr static const char* NAME = node::ToFBase::NAME;
    ToFBase() : ToFBase(std::make_unique<dai::ToFProperties>()) {}
    ToFBase(std::unique_ptr<ToFProperties> props) : dai::node::ToFBase(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;

   private:
    bool initialize(const std::shared_ptr<ImgFrame>& inImage,
                    std::unique_ptr<tofVD55H1>& vd55h1,
                    bool& wantAmplitude,
                    bool& wantConfidence,
                    bool& wantIntensity,
                    dai::ImgTransformation& localTransform,
                    std::shared_ptr<Camera>& sourceCamera,
                    void*& sensorErrorCbHandle);
};

}  // namespace gate
}  // namespace dai
