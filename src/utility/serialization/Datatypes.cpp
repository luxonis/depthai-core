#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/pipeline/datatype/AprilTagConfig.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"
#include "depthai/pipeline/datatype/AutoCalibrationResult.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/DetectionParserConfig.hpp"
#ifndef RVC2_FW
    #include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
    #include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#endif
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MapData.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"
#include "depthai/pipeline/datatype/Odometry.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/RGBDData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"
#include "depthai/pipeline/datatype/ThermalConfig.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "utility/serialization/SerializationDefine.hpp"

namespace dai {

DEPTHAI_SERIALIZE_DEFINE(AlignConfig)
DEPTHAI_SERIALIZE_DEFINE(AprilTagConfig::QuadThresholds)
DEPTHAI_SERIALIZE_DEFINE(AprilTagConfig)
DEPTHAI_SERIALIZE_EXT_DEFINE(AprilTag)
DEPTHAI_SERIALIZE_DEFINE(AprilTags)
DEPTHAI_SERIALIZE_DEFINE(AutoCalibrationConfig)
DEPTHAI_SERIALIZE_DEFINE(AutoCalibrationResult)
DEPTHAI_SERIALIZE_DEFINE(BenchmarkReport)
DEPTHAI_SERIALIZE_DEFINE(Buffer)
DEPTHAI_SERIALIZE_DEFINE(CameraControl::ManualExposureParams)
DEPTHAI_SERIALIZE_DEFINE(CameraControl::RegionParams)
DEPTHAI_SERIALIZE_DEFINE(CameraControl::StrobeTimings)
DEPTHAI_SERIALIZE_DEFINE(CameraControl::StrobeConfig)
DEPTHAI_SERIALIZE_DEFINE(CameraControl)
DEPTHAI_SERIALIZE_DEFINE(DetectionParserConfig)
#ifndef RVC2_FW
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::Calibrate)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::ComputeCalibrationMetrics)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::CalibrationQuality)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::StartCalibration)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::ApplyCalibration)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationControl::Commands::SetPerformanceMode)
DEPTHAI_SERIALIZE_DEFINE(CoverageData)
DEPTHAI_SERIALIZE_DEFINE(CalibrationQuality::Data)
DEPTHAI_SERIALIZE_DEFINE(CalibrationQuality)
DEPTHAI_SERIALIZE_DEFINE(CalibrationMetrics)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationResult::Data)
DEPTHAI_SERIALIZE_DEFINE(DynamicCalibrationResult)
#endif
DEPTHAI_SERIALIZE_DEFINE(EdgeDetectorConfig::EdgeDetectorConfigData)
DEPTHAI_SERIALIZE_DEFINE(EdgeDetectorConfig)
DEPTHAI_SERIALIZE_DEFINE(EncodedFrame)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig::CornerDetector::Thresholds)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig::CornerDetector)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig::MotionEstimator::OpticalFlow)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig::MotionEstimator)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig::FeatureMaintainer)
DEPTHAI_SERIALIZE_DEFINE(FeatureTrackerConfig)
DEPTHAI_SERIALIZE_DEFINE(GPUStereoConfig)
DEPTHAI_SERIALIZE_DEFINE(GateControl)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUReport)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUReportAccelerometer)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUReportGyroscope)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUReportMagneticField)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUReportRotationVectorWAcc)
DEPTHAI_SERIALIZE_EXT_DEFINE(IMUPacket)
DEPTHAI_SERIALIZE_DEFINE(IMUData)
DEPTHAI_SERIALIZE_DEFINE(ImageAlignConfig)
DEPTHAI_SERIALIZE_DEFINE(ImageFiltersConfig)
DEPTHAI_SERIALIZE_DEFINE(ToFDepthConfidenceFilterConfig)
DEPTHAI_SERIALIZE_DEFINE(Translate)
DEPTHAI_SERIALIZE_DEFINE(Rotate)
DEPTHAI_SERIALIZE_DEFINE(Resize)
DEPTHAI_SERIALIZE_DEFINE(Flip)
DEPTHAI_SERIALIZE_DEFINE(Affine)
DEPTHAI_SERIALIZE_DEFINE(Perspective)
DEPTHAI_SERIALIZE_DEFINE(FourPoints)
DEPTHAI_SERIALIZE_DEFINE(Crop)
DEPTHAI_SERIALIZE_DEFINE(CropRotated)
DEPTHAI_SERIALIZE_DEFINE(ManipOp)
DEPTHAI_SERIALIZE_DEFINE(ImageManipConfig)
DEPTHAI_SERIALIZE_EXT_DEFINE(CircleAnnotation)
DEPTHAI_SERIALIZE_EXT_DEFINE(PointsAnnotation)
DEPTHAI_SERIALIZE_EXT_DEFINE(TextAnnotation)
DEPTHAI_SERIALIZE_EXT_DEFINE(ImgAnnotation)
DEPTHAI_SERIALIZE_DEFINE(ImgAnnotations)
DEPTHAI_SERIALIZE_DEFINE(ImgDetection)
DEPTHAI_SERIALIZE_DEFINE(ImgDetections)
DEPTHAI_SERIALIZE_DEFINE(ImgDetectionsFilterConfig)
DEPTHAI_SERIALIZE_DEFINE(ImgFrame::Specs)
DEPTHAI_SERIALIZE_DEFINE(ImgFrame::CameraSettings)
DEPTHAI_SERIALIZE_DEFINE(ImgFrame)
DEPTHAI_SERIALIZE_DEFINE(MapData)
DEPTHAI_SERIALIZE_DEFINE(MessageGroup)
DEPTHAI_SERIALIZE_DEFINE(NNData)
DEPTHAI_SERIALIZE_DEFINE(NeuralDepthConfig::AlgorithmControl)
DEPTHAI_SERIALIZE_DEFINE(NeuralDepthConfig::PostProcessing)
DEPTHAI_SERIALIZE_DEFINE(NeuralDepthConfig)
DEPTHAI_SERIALIZE_DEFINE(ObjectTrackerConfig)
DEPTHAI_SERIALIZE_DEFINE(Odometry)
DEPTHAI_SERIALIZE_DEFINE(PipelineEvent)
DEPTHAI_SERIALIZE_DEFINE(NodeEventAggregationConfig)
DEPTHAI_SERIALIZE_DEFINE(PipelineEventAggregationConfig)
DEPTHAI_SERIALIZE_DEFINE(NodeState::DurationEvent)
DEPTHAI_SERIALIZE_DEFINE(NodeState::DurationStats)
DEPTHAI_SERIALIZE_DEFINE(NodeState::Timing)
DEPTHAI_SERIALIZE_DEFINE(NodeState::QueueStats)
DEPTHAI_SERIALIZE_DEFINE(NodeState::InputQueueState)
DEPTHAI_SERIALIZE_DEFINE(NodeState::OutputQueueState)
DEPTHAI_SERIALIZE_DEFINE(NodeState)
DEPTHAI_SERIALIZE_DEFINE(PipelineState)
DEPTHAI_SERIALIZE_DEFINE(PointCloudConfig)
DEPTHAI_SERIALIZE_DEFINE(PointCloudData)
DEPTHAI_SERIALIZE_DEFINE(RGBDData)
DEPTHAI_SERIALIZE_DEFINE(SegmentationMask)
DEPTHAI_SERIALIZE_DEFINE(SegmentationParserConfig)
DEPTHAI_SERIALIZE_DEFINE(SpatialImgDetection)
DEPTHAI_SERIALIZE_DEFINE(SpatialImgDetections)
DEPTHAI_SERIALIZE_EXT_DEFINE(SpatialLocationCalculatorConfigThresholds)
DEPTHAI_SERIALIZE_EXT_DEFINE(SpatialLocationCalculatorConfigData)
DEPTHAI_SERIALIZE_DEFINE(SpatialLocationCalculatorConfig)
DEPTHAI_SERIALIZE_EXT_DEFINE(SpatialLocations)
DEPTHAI_SERIALIZE_DEFINE(SpatialLocationCalculatorData)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::AlgorithmControl)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::ConfidenceMetrics)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::PostProcessing::BrightnessFilter)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::PostProcessing::DecimationFilter)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::PostProcessing::HoleFilling)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::PostProcessing::AdaptiveMedianFilter)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::PostProcessing)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CensusTransform)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CostMatching::LinearEquationParameters)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CostMatching)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CostAggregation::P1Config)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CostAggregation::P2Config)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig::CostAggregation)
DEPTHAI_SERIALIZE_DEFINE(StereoDepthConfig)
DEPTHAI_SERIALIZE_DEFINE(SystemInformation)
DEPTHAI_SERIALIZE_DEFINE(SystemInformationRVC4)
DEPTHAI_SERIALIZE_DEFINE(ThermalConfig::ThermalAmbientParams)
DEPTHAI_SERIALIZE_DEFINE(ThermalConfig::ThermalFFCParams)
DEPTHAI_SERIALIZE_DEFINE(ThermalConfig::ThermalImageParams)
DEPTHAI_SERIALIZE_DEFINE(ThermalConfig)
DEPTHAI_SERIALIZE_DEFINE(ToFConfig)
DEPTHAI_SERIALIZE_EXT_DEFINE(TrackedFeature)
DEPTHAI_SERIALIZE_DEFINE(TrackedFeatures)
DEPTHAI_SERIALIZE_DEFINE(Tracklet)
DEPTHAI_SERIALIZE_DEFINE(Tracklets)
DEPTHAI_SERIALIZE_EXT_DEFINE(Transform)
DEPTHAI_SERIALIZE_DEFINE(TransformData)
DEPTHAI_SERIALIZE_DEFINE(Transformable)
DEPTHAI_SERIALIZE_DEFINE(TransformableBuffer)
DEPTHAI_SERIALIZE_DEFINE(VppConfig::InjectionParameters)
DEPTHAI_SERIALIZE_DEFINE(VppConfig)

}  // namespace dai
