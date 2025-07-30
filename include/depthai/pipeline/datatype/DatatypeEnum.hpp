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
    DynamicCalibrationConfig,
    DynamicCalibrationCommand,
    DynamicCalibrationResult,
    CalibrationQuality,
    CoverageData,
    FeatureTrackerConfig,
    ThermalConfig,
    ToFConfig,
    TrackedFeatures,
    BenchmarkReport,
    MessageGroup,
    TransformData,
    PointCloudConfig,
    PointCloudData,
    RGBDData,
    ImageAlignConfig,
    ImgAnnotations,
    ObjectTrackerConfig,
};
bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children);

}  // namespace dai
