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
    FeatureTrackerConfig,
    ToFConfig,
    TrackedFeatures,
    BenchmarkReport,
    MessageGroup,
    TransformData,
    PointCloudConfig,
    PointCloudData
};
bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children);

}  // namespace dai
