#include "pipeline/datatype/DatatypeEnum.hpp"

#include <functional>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace dai {

const std::unordered_map<DatatypeEnum, std::vector<DatatypeEnum>> hierarchy = {
    {DatatypeEnum::ADatatype,
     {
         DatatypeEnum::Buffer,
         DatatypeEnum::ImgFrame,
         DatatypeEnum::EncodedFrame,
         DatatypeEnum::NNData,
         DatatypeEnum::ImageManipConfig,
         DatatypeEnum::CameraControl,
         DatatypeEnum::ImgDetections,
         DatatypeEnum::ImageAlignConfig,
         DatatypeEnum::SpatialImgDetections,
         DatatypeEnum::SystemInformation,
         DatatypeEnum::SystemInformationS3,
         DatatypeEnum::SpatialLocationCalculatorConfig,
         DatatypeEnum::SpatialLocationCalculatorData,
         DatatypeEnum::EdgeDetectorConfig,
         DatatypeEnum::Tracklets,
         DatatypeEnum::IMUData,
         DatatypeEnum::StereoDepthConfig,
         DatatypeEnum::NeuralDepthConfig,
         DatatypeEnum::FeatureTrackerConfig,
         DatatypeEnum::ThermalConfig,
         DatatypeEnum::ToFConfig,
         DatatypeEnum::TrackedFeatures,
         DatatypeEnum::AprilTagConfig,
         DatatypeEnum::AprilTags,
         DatatypeEnum::BenchmarkReport,
         DatatypeEnum::MessageGroup,
         DatatypeEnum::PointCloudConfig,
         DatatypeEnum::PointCloudData,
         DatatypeEnum::RGBDData,
         DatatypeEnum::TransformData,
         DatatypeEnum::ImgAnnotations,
         DatatypeEnum::VppConfig,
         DatatypeEnum::ImageFiltersConfig,
         DatatypeEnum::ToFDepthConfidenceFilterConfig,
         DatatypeEnum::ObjectTrackerConfig,
         DatatypeEnum::DynamicCalibrationControl,
         DatatypeEnum::DynamicCalibrationResult,
         DatatypeEnum::CoverageData,
         DatatypeEnum::CalibrationQuality,

     }},
    {DatatypeEnum::Buffer,
     {
         DatatypeEnum::ImgFrame,
         DatatypeEnum::EncodedFrame,
         DatatypeEnum::NNData,
         DatatypeEnum::ImageManipConfig,
         DatatypeEnum::CameraControl,
         DatatypeEnum::ImgDetections,
         DatatypeEnum::ImageAlignConfig,
         DatatypeEnum::SpatialImgDetections,
         DatatypeEnum::SystemInformation,
         DatatypeEnum::SystemInformationS3,
         DatatypeEnum::SpatialLocationCalculatorConfig,
         DatatypeEnum::SpatialLocationCalculatorData,
         DatatypeEnum::EdgeDetectorConfig,
         DatatypeEnum::Tracklets,
         DatatypeEnum::IMUData,
         DatatypeEnum::StereoDepthConfig,
         DatatypeEnum::NeuralDepthConfig,
         DatatypeEnum::FeatureTrackerConfig,
         DatatypeEnum::ThermalConfig,
         DatatypeEnum::ToFConfig,
         DatatypeEnum::TrackedFeatures,
         DatatypeEnum::AprilTagConfig,
         DatatypeEnum::AprilTags,
         DatatypeEnum::BenchmarkReport,
         DatatypeEnum::MessageGroup,
         DatatypeEnum::PointCloudConfig,
         DatatypeEnum::PointCloudData,
         DatatypeEnum::RGBDData,
         DatatypeEnum::TransformData,
         DatatypeEnum::ImgAnnotations,
         DatatypeEnum::ImageFiltersConfig,
         DatatypeEnum::VppConfig,
         DatatypeEnum::ToFDepthConfidenceFilterConfig,
         DatatypeEnum::ObjectTrackerConfig,
         DatatypeEnum::DynamicCalibrationControl,
         DatatypeEnum::DynamicCalibrationResult,
         DatatypeEnum::CoverageData,
         DatatypeEnum::CalibrationQuality,
     }},
    {DatatypeEnum::ImgFrame, {}},
    {DatatypeEnum::EncodedFrame, {}},
    {DatatypeEnum::NNData, {}},
    {DatatypeEnum::ImageManipConfig, {}},
    {DatatypeEnum::ImageAlignConfig, {}},
    {DatatypeEnum::CameraControl, {}},
    {DatatypeEnum::ImgDetections, {DatatypeEnum::SpatialImgDetections}},
    {DatatypeEnum::SpatialImgDetections, {}},
    {DatatypeEnum::SystemInformation, {}},
    {DatatypeEnum::SystemInformationS3, {}},
    {DatatypeEnum::SpatialLocationCalculatorConfig, {}},
    {DatatypeEnum::SpatialLocationCalculatorData, {}},
    {DatatypeEnum::EdgeDetectorConfig, {}},
    {DatatypeEnum::Tracklets, {}},
    {DatatypeEnum::IMUData, {}},
    {DatatypeEnum::StereoDepthConfig, {}},
    {DatatypeEnum::NeuralDepthConfig, {}},
    {DatatypeEnum::FeatureTrackerConfig, {}},
    {DatatypeEnum::ThermalConfig, {}},
    {DatatypeEnum::ToFConfig, {}},
    {DatatypeEnum::TrackedFeatures, {}},
    {DatatypeEnum::AprilTagConfig, {}},
    {DatatypeEnum::AprilTags, {}},
    {DatatypeEnum::BenchmarkReport, {}},
    {DatatypeEnum::MessageGroup, {}},
    {DatatypeEnum::PointCloudConfig, {}},
    {DatatypeEnum::PointCloudData, {}},
    {DatatypeEnum::RGBDData, {}},
    {DatatypeEnum::TransformData, {}},
    {DatatypeEnum::ImgAnnotations, {}},
    {DatatypeEnum::VppConfig, {}},
    {DatatypeEnum::ImageFiltersConfig, {}},
    {DatatypeEnum::ToFDepthConfidenceFilterConfig, {}},
    {DatatypeEnum::ObjectTrackerConfig, {}},
    {DatatypeEnum::DynamicCalibrationControl, {}},
    {DatatypeEnum::DynamicCalibrationResult, {}},
    {DatatypeEnum::CoverageData, {}},
    {DatatypeEnum::CalibrationQuality, {}},
};

bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children) {
    // Check if parent is in hierarchy
    if(hierarchy.find(parent) == hierarchy.end()) {
        throw std::invalid_argument("Parent datatype not found in hierarchy");
    }
    for(const auto& d : hierarchy.at(parent)) {
        if(d == children) return true;
        if(isDatatypeSubclassOf(d, children)) return true;
    }
    return false;
}

}  // namespace dai
