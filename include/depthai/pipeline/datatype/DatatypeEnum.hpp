#pragma once

#include <cstdint>

namespace dai {

enum class DatatypeEnum : std::int32_t {
    ADatatype,
    Buffer,
    ImgFrame,
    EncodedFrame,
    NNData,
    ImageManipConfig,
    CameraControl,
    ImgDetections,
    SpatialImgDetections,
    SystemInformation,
    SystemInformationS3,
    SpatialLocationCalculatorConfig,
    SpatialLocationCalculatorData,
    EdgeDetectorConfig,
    AprilTagConfig,
    AprilTags,
    Tracklets,
    IMUData,
    StereoDepthConfig,
    NeuralDepthConfig,
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
    CalibrationQuality,
    CoverageData,
    PipelineEvent,
    PipelineState,
    PipelineEventAggregationConfig,
    VppConfig
};
bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children);

}  // namespace dai
