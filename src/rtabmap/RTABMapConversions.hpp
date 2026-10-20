#include <memory>

#include "common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/Odometry.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace rtabmap {
class Transform;
class StereoCameraModel;
class CameraModel;
}  // namespace rtabmap

namespace dai {

std::shared_ptr<TransformData> rtabmapToTransformData(rtabmap::Transform transformRTABMap);
std::shared_ptr<Odometry> rtabmapToOdometry(rtabmap::Transform transformRTABMap, const Point3d& velocity);
rtabmap::Transform getRTABMapTransform(const Transform& transform);
// Build an RGB-D model relative to the tracked frame described by odometry.
// Both image and tracked frame extrinsics must name a compatible common reference.
rtabmap::CameraModel getRTABMapFrameCameraModel(const ImgFrame& image, const ImgFrame& depth, const Extrinsics& trackedFrameExtrinsics);
rtabmap::StereoCameraModel getRTABMapCameraModel(CameraBoardSocket cameraId,
                                                 int width,
                                                 int height,
                                                 const rtabmap::Transform& localTransform,
                                                 float alphaScaling,
                                                 CalibrationHandler calHandler,
                                                 CameraBoardSocket left = CameraBoardSocket::CAM_B,
                                                 CameraBoardSocket right = CameraBoardSocket::CAM_C);
}  // namespace dai
