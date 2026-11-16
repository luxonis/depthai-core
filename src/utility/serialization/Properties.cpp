#include "depthai/beta/properties/ToFStereoFusionProperties.hpp"
#include "depthai/properties/AlignProperties.hpp"
#include "depthai/properties/AprilTagProperties.hpp"
#include "depthai/properties/BenchmarkInProperties.hpp"
#include "depthai/properties/BenchmarkOutProperties.hpp"
#include "depthai/properties/CameraProperties.hpp"
#include "depthai/properties/CastProperties.hpp"
#include "depthai/properties/ColorCameraProperties.hpp"
#include "depthai/properties/DetectionParserProperties.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"
#include "depthai/properties/EdgeDetectorProperties.hpp"
#include "depthai/properties/FeatureTrackerProperties.hpp"
#include "depthai/properties/GPUStereoProperties.hpp"
#include "depthai/properties/GateProperties.hpp"
#include "depthai/properties/GlobalProperties.hpp"
#include "depthai/properties/IMUProperties.hpp"
#include "depthai/properties/ImageAlignProperties.hpp"
#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/properties/ImgDetectionsFilterProperties.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"
#include "depthai/properties/NeuralDepthProperties.hpp"
#include "depthai/properties/NeuralNetworkProperties.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"
#include "depthai/properties/PointCloudProperties.hpp"
#include "depthai/properties/PoolProperties.hpp"
#include "depthai/properties/RectificationProperties.hpp"
#include "depthai/properties/SPIInProperties.hpp"
#include "depthai/properties/SPIOutProperties.hpp"
#include "depthai/properties/ScriptProperties.hpp"
#include "depthai/properties/SegmentationParserProperties.hpp"
#include "depthai/properties/SpatialDetectionNetworkProperties.hpp"
#include "depthai/properties/SpatialLocationCalculatorProperties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"
#include "depthai/properties/StitchingProperties.hpp"
#include "depthai/properties/SyncProperties.hpp"
#include "depthai/properties/SystemLoggerProperties.hpp"
#include "depthai/properties/ThermalProperties.hpp"
#include "depthai/properties/ToFProperties.hpp"
#include "depthai/properties/UVCProperties.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/properties/VppProperties.hpp"
#include "depthai/properties/WarpProperties.hpp"
#include "depthai/properties/internal/PipelineEventAggregationProperties.hpp"
#include "depthai/properties/internal/XLinkInProperties.hpp"
#include "depthai/properties/internal/XLinkOutProperties.hpp"
#include "utility/serialization/SerializationDefine.hpp"

// RVC2_FW does not need these properties
#ifndef RVC2_FW
    #include "depthai/properties/ImageFiltersProperties.hpp"

    #ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
        #include "depthai/properties/AutoCalibrationProperties.hpp"
        #include "depthai/properties/DynamicCalibrationProperties.hpp"
    #endif
#endif

#ifdef DEPTHAI_HAVE_BETA
    #include "depthai/beta/properties/ClassificationParserProperties.hpp"
    #include "depthai/beta/properties/ClassificationSequenceParserProperties.hpp"
    #include "depthai/beta/properties/EmbeddingsParserProperties.hpp"
    #include "depthai/beta/properties/FastSAMParserProperties.hpp"
    #include "depthai/beta/properties/HRNetParserProperties.hpp"
    #include "depthai/beta/properties/ImageOutputParserProperties.hpp"
    #include "depthai/beta/properties/KeypointParserProperties.hpp"
    #include "depthai/beta/properties/LaneDetectionParserProperties.hpp"
    #include "depthai/beta/properties/MLSDParserProperties.hpp"
    #include "depthai/beta/properties/MPPalmDetectionParserProperties.hpp"
    #include "depthai/beta/properties/MapOutputParserProperties.hpp"
    #include "depthai/beta/properties/PPTextDetectionParserProperties.hpp"
    #include "depthai/beta/properties/RFDETRParserProperties.hpp"
    #include "depthai/beta/properties/RegressionParserProperties.hpp"
    #include "depthai/beta/properties/SCRFDParserProperties.hpp"
    #include "depthai/beta/properties/SuperAnimalParserProperties.hpp"
    #include "depthai/beta/properties/XFeatMonoParserProperties.hpp"
    #include "depthai/beta/properties/XFeatStereoParserProperties.hpp"
    #include "depthai/beta/properties/YuNetParserProperties.hpp"
#endif

namespace dai {

DEPTHAI_SERIALIZE_DEFINE(ColorCameraProperties::IspScale)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUSensorConfig)
DEPTHAI_SERIALIZE_EXT_DEFINE(PoolProperties)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthProperties::RectificationMesh)
DEPTHAI_SERIALIZE_EXT_DEFINE(StitchingProperties::Plane)
DEPTHAI_SERIALIZE_EXT_DEFINE(StitchingProperties::VirtualCamera)

}  // namespace dai

// ToFStereoFusion is built also without DEPTHAI_BUILD_BETA (CMakeLists.txt), so its properties and config are not in the beta block.
namespace dai {
namespace beta {

DEPTHAI_SERIALIZE_EXT_DEFINE(ToFStereoFusionConfig)

}  // namespace beta
}  // namespace dai

DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, AlignProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, AprilTagProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, BenchmarkInProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, BenchmarkOutProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, CameraProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, CastProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ColorCameraProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, DetectionParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, DeviceNodeGroupProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, DeviceProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, EdgeDetectorProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, FeatureTrackerProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, GPUStereoProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, GateProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, GlobalProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, IMUProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ImageAlignProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ImageManipProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ImgDetectionsFilterProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, MessageDemuxProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, MonoCameraProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, NeuralDepthProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, NeuralNetworkProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ObjectTrackerProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, PipelineEventAggregationProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, PointCloudProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, RectificationProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SPIInProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SPIOutProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ScriptProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SegmentationParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SpatialDetectionNetworkProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SpatialLocationCalculatorProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, StereoDepthProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, StitchingProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SyncProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, SystemLoggerProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ThermalProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ToFProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, UVCProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, VideoEncoderProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, VppProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, WarpProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::internal, XLinkInProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::internal, XLinkOutProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, ToFStereoFusionProperties)

// RVC2_FW does not need these properties
#ifndef RVC2_FW
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ImageFiltersProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, ToFDepthConfidenceFilterProperties)

    #ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, AutoCalibrationProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai, DynamicCalibrationProperties)
    #endif
#endif

#ifdef DEPTHAI_HAVE_BETA
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, ClassificationParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, ClassificationSequenceParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, EmbeddingsParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, FastSAMParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, HRNetParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, ImageOutputParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, KeypointParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, LaneDetectionParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, MLSDParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, MPPalmDetectionParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, MapOutputParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, PPTextDetectionParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, RFDETRParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, RegressionParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, SCRFDParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, SuperAnimalParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, XFeatMonoParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, XFeatStereoParserProperties)
DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(dai::beta, YuNetParserProperties)
#endif
