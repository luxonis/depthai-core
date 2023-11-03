#include "depthai/utility/TypeToEnum.hpp"

namespace dai {

template <>
DatatypeEnum rawToType<Buffer>() {
    return DatatypeEnum::Buffer;
}
template <>
DatatypeEnum rawToType<ImgFrame>() {
    return DatatypeEnum::ImgFrame;
}
template <>
DatatypeEnum rawToType<EncodedFrame>() {
    return DatatypeEnum::EncodedFrame;
}
template <>
DatatypeEnum rawToType<NNData>() {
    return DatatypeEnum::NNData;
}
template <>
DatatypeEnum rawToType<ImageManipConfig>() {
    return DatatypeEnum::ImageManipConfig;
}
template <>
DatatypeEnum rawToType<CameraControl>() {
    return DatatypeEnum::CameraControl;
}
template <>
DatatypeEnum rawToType<ImgDetections>() {
    return DatatypeEnum::ImgDetections;
}
template <>
DatatypeEnum rawToType<SpatialImgDetections>() {
    return DatatypeEnum::SpatialImgDetections;
}
template <>
DatatypeEnum rawToType<SystemInformation>() {
    return DatatypeEnum::SystemInformation;
}
template <>
DatatypeEnum rawToType<SpatialLocationCalculatorConfig>() {
    return DatatypeEnum::SpatialLocationCalculatorConfig;
}
template <>
DatatypeEnum rawToType<SpatialLocationCalculatorData>() {
    return DatatypeEnum::SpatialLocationCalculatorData;
}
template <>
DatatypeEnum rawToType<EdgeDetectorConfig>() {
    return DatatypeEnum::EdgeDetectorConfig;
}
template <>
DatatypeEnum rawToType<AprilTagConfig>() {
    return DatatypeEnum::AprilTagConfig;
}
template <>
DatatypeEnum rawToType<AprilTags>() {
    return DatatypeEnum::AprilTags;
}
template <>
DatatypeEnum rawToType<Tracklets>() {
    return DatatypeEnum::Tracklets;
}
template <>
DatatypeEnum rawToType<IMUData>() {
    return DatatypeEnum::IMUData;
}
template <>
DatatypeEnum rawToType<StereoDepthConfig>() {
    return DatatypeEnum::StereoDepthConfig;
}
template <>
DatatypeEnum rawToType<FeatureTrackerConfig>() {
    return DatatypeEnum::FeatureTrackerConfig;
}
template <>
DatatypeEnum rawToType<ToFConfig>() {
    return DatatypeEnum::ToFConfig;
}
template <>
DatatypeEnum rawToType<TrackedFeatures>() {
    return DatatypeEnum::TrackedFeatures;
}

}  // namespace dai
