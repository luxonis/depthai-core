#pragma once
#include <spdlog/async_logger.h>

#include <optional>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/properties/DetectionParserProperties.hpp"

namespace dai {
namespace utilities {
namespace DetectionParserUtils {

constexpr std::size_t defaultMaxDetectionsPerFrame = 250;
struct DetectionCandidate {
    float xmin, ymin, xmax, ymax, score;
    int label, headIndex, rowIndex, columnIndex;
    std::optional<std::string> labelName;
};
/**
Decode anchor free yolo v6r1 with sigmoid assisted center detection
*/
void decodeR1AF(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger);

/**
Decode anchor based yolo v3 and v3-Tiny
*/
void decodeV3AB(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger);

/**
Decode anchor based networks, e.g., yolo v5, v7, P
*/
void decodeV5AB(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger);

/**
Decode anchor free top-left-bottom-right (TLBR) style networks, e.g., yolo v6r2, v8, v10, v11
*/
void decodeTLBR(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger);

std::vector<std::string> getSortedDetectionLayerNames(const dai::NNData& nnData, std::string searchTerm, std::vector<std::string> outputNames);

float YoloIntersectionOverUnion(const DetectionCandidate& box1, const DetectionCandidate& box2);

bool isTensorOrderValid(dai::TensorInfo& tensorInfo, DetectionParserProperties properties, std::shared_ptr<spdlog::async_logger>& logger);

void createImgDetections(std::vector<DetectionCandidate>& detectionCandidates,
                         std::vector<int> keepIndices,
                         dai::ImgDetections& outDetections,
                         std::shared_ptr<spdlog::async_logger>& logger);

std::vector<DetectionCandidate> nonMaximumSuppression(std::vector<DetectionCandidate>& detectionCandidates, float iouThr);

void createImgDetections(const std::vector<DetectionCandidate>& detectionCandidates,
                         dai::ImgDetections& outDetections,
                         unsigned int width,
                         unsigned int height);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
void segmentationDecode(const dai::NNData& nnData,
                        std::vector<DetectionCandidate>& detectionCandidates,
                        dai::ImgDetections& outDetections,
                        DetectionParserProperties properties,
                        std::shared_ptr<spdlog::async_logger>& logger);
#endif

void keypointDecode(const dai::NNData& nnData,
                    std::vector<DetectionCandidate>& detectionCandidates,
                    dai::ImgDetections& outDetections,
                    DetectionParserProperties properties,
                    std::shared_ptr<spdlog::async_logger>& logger);

}  // namespace DetectionParserUtils
}  // namespace utilities
}  // namespace dai