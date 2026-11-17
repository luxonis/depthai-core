#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraInfo.hpp"
#include "depthai/common/ChipTemperature.hpp"
#include "depthai/common/ChipTemperatureRVC4.hpp"
#include "depthai/common/Color.hpp"
#include "depthai/common/CpuUsage.hpp"
#include "depthai/common/DetectionParserOptions.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/ImuParameters.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/MemoryInfo.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3d.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/common/Quaterniond.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/common/StereoRectification.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/common/WorldConstraints.hpp"
#include "depthai/log/LogMessage.hpp"
#include "utility/serialization/SerializationDefine.hpp"

namespace dai {

DEPTHAI_SERIALIZE_DEFINE(ImgFrameCapability)
DEPTHAI_SERIALIZE_EXT_DEFINE(CameraSensorConfig)
DEPTHAI_SERIALIZE_DEFINE(CameraFeatures)
DEPTHAI_SERIALIZE_DEFINE(CameraInfo)
DEPTHAI_SERIALIZE_EXT_DEFINE(ChipTemperature)
DEPTHAI_SERIALIZE_EXT_DEFINE(ChipTemperatureRVC4)
DEPTHAI_SERIALIZE_EXT_DEFINE(Color)
DEPTHAI_SERIALIZE_EXT_DEFINE(CpuUsage)
DEPTHAI_SERIALIZE_EXT_DEFINE(DetectionParserOptions)
DEPTHAI_SERIALIZE_OPTIONAL_EXT_DEFINE(EepromData)
DEPTHAI_SERIALIZE_OPTIONAL_DEFINE(Extrinsics)
DEPTHAI_SERIALIZE_EXT_DEFINE(MultiDeviceExtrinsics)
DEPTHAI_SERIALIZE_DEFINE(ImgTransformation)
DEPTHAI_SERIALIZE_DEFINE(AccelAxisNoiseParams)
DEPTHAI_SERIALIZE_DEFINE(GyroAxisNoiseParams)
DEPTHAI_SERIALIZE_DEFINE(AccelerometerNoiseParams)
DEPTHAI_SERIALIZE_DEFINE(GyroscopeNoiseParams)
DEPTHAI_SERIALIZE_DEFINE(ImuNoiseParameters)
DEPTHAI_SERIALIZE_DEFINE(ImuCalibrationParams)
DEPTHAI_SERIALIZE_DEFINE(Keypoint)
DEPTHAI_SERIALIZE_DEFINE(KeypointsList)
DEPTHAI_SERIALIZE_EXT_DEFINE(MemoryInfo)
DEPTHAI_SERIALIZE_EXT_DEFINE(Point2f)
DEPTHAI_SERIALIZE_EXT_DEFINE(Point3d)
DEPTHAI_SERIALIZE_EXT_DEFINE(Point3f)
DEPTHAI_SERIALIZE_EXT_DEFINE(Point3fRGBA)
DEPTHAI_SERIALIZE_EXT_DEFINE(Quaterniond)
DEPTHAI_SERIALIZE_EXT_DEFINE(Rect)
DEPTHAI_SERIALIZE_EXT_DEFINE(RotatedRect)
DEPTHAI_SERIALIZE_EXT_DEFINE(Size2f)
DEPTHAI_SERIALIZE_DEFINE(SpatialKeypoint)
DEPTHAI_SERIALIZE_DEFINE(SpatialKeypointsList)
DEPTHAI_SERIALIZE_DEFINE(StereoPair)
DEPTHAI_SERIALIZE_EXT_DEFINE(StereoRectification)
DEPTHAI_SERIALIZE_EXT_DEFINE(TensorInfo)
DEPTHAI_SERIALIZE_EXT_DEFINE(Timestamp)
DEPTHAI_SERIALIZE_DEFINE(WorldConstraints::MarkerDistance)
DEPTHAI_SERIALIZE_DEFINE(WorldConstraints::MarkerSocketDistance)
DEPTHAI_SERIALIZE_DEFINE(WorldConstraints::MarkerSize)
DEPTHAI_SERIALIZE_DEFINE(WorldConstraints)
DEPTHAI_SERIALIZE_DEFINE(MarkerOutline)
DEPTHAI_SERIALIZE_DEFINE(MarkerObservationSet)
DEPTHAI_SERIALIZE_EXT_DEFINE(LogMessage)

}  // namespace dai
