#include "depthai/properties/Properties.hpp"

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
#include "depthai/properties/GlobalProperties.hpp"
#include "depthai/properties/IMUProperties.hpp"
#include "depthai/properties/ImageAlignProperties.hpp"
#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"
#include "depthai/properties/NeuralDepthProperties.hpp"
#include "depthai/properties/NeuralNetworkProperties.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"
#include "depthai/properties/PointCloudProperties.hpp"
#include "depthai/properties/RectificationProperties.hpp"
#include "depthai/properties/SPIInProperties.hpp"
#include "depthai/properties/SPIOutProperties.hpp"
#include "depthai/properties/ScriptProperties.hpp"
#include "depthai/properties/SegmentationParserProperties.hpp"
#include "depthai/properties/SpatialDetectionNetworkProperties.hpp"
#include "depthai/properties/SpatialLocationCalculatorProperties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"
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

// RVC2_FW does not need these properties
#ifndef RVC2_FW
    #include "depthai/properties/ImageFiltersProperties.hpp"

    #ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
        #include "depthai/properties/DynamicCalibrationProperties.hpp"
    #endif
#endif

namespace dai {

Properties::~Properties() = default;

namespace internal {
XLinkInProperties::~XLinkInProperties() = default;
XLinkOutProperties::~XLinkOutProperties() = default;
}  // namespace internal

AprilTagProperties::~AprilTagProperties() = default;
BenchmarkInProperties::~BenchmarkInProperties() = default;
BenchmarkOutProperties::~BenchmarkOutProperties() = default;
CameraProperties::~CameraProperties() = default;
ColorCameraProperties::~ColorCameraProperties() = default;
DetectionParserProperties::~DetectionParserProperties() = default;
EdgeDetectorProperties::~EdgeDetectorProperties() = default;
RectificationProperties::~RectificationProperties() = default;
NeuralDepthProperties::~NeuralDepthProperties() = default;
FeatureTrackerProperties::~FeatureTrackerProperties() = default;
IMUProperties::~IMUProperties() = default;
ImageAlignProperties::~ImageAlignProperties() = default;
ImageManipProperties::~ImageManipProperties() = default;
DeviceNodeGroupProperties::~DeviceNodeGroupProperties() = default;
MessageDemuxProperties::~MessageDemuxProperties() = default;
MonoCameraProperties::~MonoCameraProperties() = default;
NeuralNetworkProperties::~NeuralNetworkProperties() = default;
ObjectTrackerProperties::~ObjectTrackerProperties() = default;
PipelineEventAggregationProperties::~PipelineEventAggregationProperties() = default;
PointCloudProperties::~PointCloudProperties() = default;
SPIInProperties::~SPIInProperties() = default;
SPIOutProperties::~SPIOutProperties() = default;
ScriptProperties::~ScriptProperties() = default;
SpatialDetectionNetworkProperties::~SpatialDetectionNetworkProperties() = default;
SpatialLocationCalculatorProperties::~SpatialLocationCalculatorProperties() = default;
SegmentationParserProperties::~SegmentationParserProperties() = default;
StereoDepthProperties::~StereoDepthProperties() = default;
SyncProperties::~SyncProperties() = default;
SystemLoggerProperties::~SystemLoggerProperties() = default;
ThermalProperties::~ThermalProperties() = default;
ToFProperties::~ToFProperties() = default;
UVCProperties::~UVCProperties() = default;
VideoEncoderProperties::~VideoEncoderProperties() = default;
WarpProperties::~WarpProperties() = default;
GlobalProperties::~GlobalProperties() = default;
CastProperties::~CastProperties() = default;
VppProperties::~VppProperties() = default;

// RVC2_FW does not need these properties
#ifndef RVC2_FW

ImageFiltersProperties::~ImageFiltersProperties() = default;
ToFDepthConfidenceFilterProperties::~ToFDepthConfidenceFilterProperties() = default;

    #ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
DynamicCalibrationProperties::~DynamicCalibrationProperties() = default;
    #endif

#endif

}  // namespace dai
