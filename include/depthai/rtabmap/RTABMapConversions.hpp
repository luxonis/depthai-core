#include <memory>

#include "common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace rtabmap {
class Transform;
class StereoCameraModel;
}  // namespace rtabmap

namespace dai {

std::shared_ptr<TransformData> rtabmapToTransformData(rtabmap::Transform transformRTABMap);
rtabmap::Transform getRTABMapTransform(const Transform& transform);
rtabmap::StereoCameraModel getRTABMapCameraModel(CameraBoardSocket cameraId,
                                                 int width,
                                                 int height,
                                                 const rtabmap::Transform& localTransform,
                                                 float alphaScaling,
                                                 CalibrationHandler calHandler,
                                                 CameraBoardSocket left = CameraBoardSocket::CAM_B,
                                                 CameraBoardSocket right = CameraBoardSocket::CAM_C);
}  // namespace dai
