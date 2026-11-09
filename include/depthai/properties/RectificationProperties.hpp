#pragma once

#include "depthai/common/Backend.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"

namespace dai {

struct RectificationProperties : PropertiesSerializable<Properties, RectificationProperties> {
    using Backend = dai::Backend;

    std::optional<uint32_t> outputWidth, outputHeight;
    bool enableRectification = true;
    Backend backend = Backend::AUTO;
    // RVC2 warp configuration, also used by the StereoDepth rectification subnode.
    StereoDepthProperties::RectificationMesh mesh;
    std::int32_t rectifyEdgeFillColor = 0;
    std::optional<bool> useHomographyRectification;
    std::optional<bool> rectificationUseSpecTranslation;
    std::optional<float> alphaScaling;
    StereoDepthProperties::DepthAlign depthAlign = StereoDepthProperties::DepthAlign::RECTIFIED_LEFT;
    CameraBoardSocket depthAlignCamera = CameraBoardSocket::AUTO;
    std::int32_t numFramesPool = 4;
    ~RectificationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RectificationProperties,
                      outputWidth,
                      outputHeight,
                      enableRectification,
                      backend,
                      mesh,
                      rectifyEdgeFillColor,
                      useHomographyRectification,
                      rectificationUseSpecTranslation,
                      alphaScaling,
                      depthAlign,
                      depthAlignCamera,
                      numFramesPool);

}  // namespace dai
