#pragma once

#include <cstdint>

namespace dai {

enum class DatatypeEnum : std::int32_t {
    ADatatype,
    Buffer,
    ImgFrame,
    EncodedFrame,
    SegmentationMask,
    GateControl,
    NNData,
    ImageManipConfig,
    CameraControl,
    ImgDetections,
    SpatialImgDetections,
    SystemInformation,
    SystemInformationRVC4,
    SpatialLocationCalculatorConfig,
    SpatialLocationCalculatorData,
    EdgeDetectorConfig,
    AprilTagConfig,
    AprilTags,
    Tracklets,
    IMUData,
    StereoDepthConfig,
    NeuralDepthConfig,
    GPUStereoConfig,
    FeatureTrackerConfig,
    ThermalConfig,
    ToFConfig,
    TrackedFeatures,
    BenchmarkReport,
    MessageGroup,
    MapData,
    TransformData,
    PointCloudConfig,
    PointCloudData,
    RGBDData,
    ImageAlignConfig,
    ImgAnnotations,
    ImageFiltersConfig,
    ToFDepthConfidenceFilterConfig,
    ObjectTrackerConfig,
    DynamicCalibrationControl,
    DynamicCalibrationResult,
    AutoCalibrationConfig,
    AutoCalibrationResult,
    CalibrationQuality,
    CalibrationMetrics,
    CoverageData,
    SegmentationParserConfig,
    PipelineEvent,
    PipelineState,
    PipelineEventAggregationConfig,
    VppConfig,
    PacketizedData,
    COUNT  // Sentinel used by consistency checks; must remain the last enum entry.
};
bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children);

}  // namespace dai
