#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"
#include "depthai/pipeline/datatype/AutoCalibrationResult.hpp"
#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"

namespace {

template <typename T>
bool writeSchema(const std::filesystem::path& outputDirectory, std::string_view datatypeName) {
    const auto outputPath = outputDirectory / (std::string{datatypeName} + ".json");
    std::ofstream output{outputPath};
    if(!output) {
        std::cerr << "Unable to open " << outputPath << " for writing\n";
        return false;
    }

    output << dai::utility::getJsonSchema<T>() << '\n';
    if(!output) {
        std::cerr << "Unable to write " << outputPath << '\n';
        return false;
    }

    return true;
}

#define WRITE_SCHEMA(Type) success = writeSchema<dai::Type>(outputDirectory, #Type) && success

}  // namespace

int main(int argc, char** argv) {
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <output-directory>\n";
        return 1;
    }

    const std::filesystem::path outputDirectory{argv[1]};
    std::error_code error;
    std::filesystem::create_directories(outputDirectory, error);
    if(error) {
        std::cerr << "Unable to create " << outputDirectory << ": " << error.message() << '\n';
        return 1;
    }

    bool success = true;
    WRITE_SCHEMA(AprilTagConfig);
    WRITE_SCHEMA(AprilTags);
    WRITE_SCHEMA(AutoCalibrationConfig);
    WRITE_SCHEMA(AutoCalibrationResult);
    WRITE_SCHEMA(BenchmarkReport);
    WRITE_SCHEMA(Buffer);
    WRITE_SCHEMA(CameraControl);
    WRITE_SCHEMA(EdgeDetectorConfig);
    WRITE_SCHEMA(EncodedFrame);
    WRITE_SCHEMA(FeatureTrackerConfig);
    WRITE_SCHEMA(GateControl);
    WRITE_SCHEMA(GPUStereoConfig);
    WRITE_SCHEMA(IMUData);
    WRITE_SCHEMA(ImageAlignConfig);
    WRITE_SCHEMA(ImageFiltersConfig);
    WRITE_SCHEMA(ImageManipConfig);
    WRITE_SCHEMA(ImgAnnotations);
    WRITE_SCHEMA(ImgDetections);
    WRITE_SCHEMA(ImgFrame);
    WRITE_SCHEMA(MapData);
    WRITE_SCHEMA(MessageGroup);
    WRITE_SCHEMA(NeuralDepthConfig);
    WRITE_SCHEMA(NNData);
    WRITE_SCHEMA(ObjectTrackerConfig);
    WRITE_SCHEMA(PipelineEvent);
    WRITE_SCHEMA(PipelineEventAggregationConfig);
    WRITE_SCHEMA(PipelineState);
    WRITE_SCHEMA(PointCloudConfig);
    WRITE_SCHEMA(PointCloudData);
    WRITE_SCHEMA(RGBDData);
    WRITE_SCHEMA(SegmentationMask);
    WRITE_SCHEMA(SegmentationParserConfig);
    WRITE_SCHEMA(SpatialImgDetections);
    WRITE_SCHEMA(SpatialLocationCalculatorConfig);
    WRITE_SCHEMA(SpatialLocationCalculatorData);
    WRITE_SCHEMA(StereoDepthConfig);
    WRITE_SCHEMA(SystemInformation);
    WRITE_SCHEMA(SystemInformationRVC4);
    WRITE_SCHEMA(ThermalConfig);
    WRITE_SCHEMA(ToFConfig);
    WRITE_SCHEMA(ToFDepthConfidenceFilterConfig);
    WRITE_SCHEMA(TrackedFeatures);
    WRITE_SCHEMA(Tracklets);
    WRITE_SCHEMA(TransformData);
    WRITE_SCHEMA(Transformable);
    WRITE_SCHEMA(VppConfig);
#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
    WRITE_SCHEMA(DynamicCalibrationControl);
    WRITE_SCHEMA(DynamicCalibrationResult);
#endif

    return success ? 0 : 1;
}
