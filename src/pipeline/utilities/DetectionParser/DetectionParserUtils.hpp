#pragma once

#include <spdlog/async_logger.h>

#include <optional>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/properties/DetectionParserProperties.hpp"

namespace dai {
namespace utilities {
namespace DetectionParserUtils {

struct DetectionCandidate {
    float xmin, ymin, xmax, ymax, score;
    int label, headIndex, rowIndex, columnIndex;
    std::optional<std::string> labelName;
};

/*
Decode anchor free yolo v6r1 with sigmoid assisted center detection
*/
void decodeR1AF(std::shared_ptr<dai::NNData> nnData,
                std::shared_ptr<dai::ImgDetections> outDetections,
                DetectionParserProperties properties,
                std::shared_ptr<spdlog::async_logger> logger);

/*
Decode anchor based yolo v3 and v3-Tiny
*/
void decodeV3AB(std::shared_ptr<dai::NNData> nnData,
                std::shared_ptr<dai::ImgDetections> outDetections,
                DetectionParserProperties properties,
                std::shared_ptr<spdlog::async_logger> logger);

/*
Decode anchor based networks, e.g., yolo v5, v7, P
*/
void decodeV5AB(std::shared_ptr<dai::NNData> nnData,
                std::shared_ptr<dai::ImgDetections> outDetections,
                DetectionParserProperties properties,
                std::shared_ptr<spdlog::async_logger> logger);

/*
Decode anchor free top-left-bottom-right (TLBR) style networks, e.g., yolo v6r2, v8, v10, v11
*/
void decodeTLBR(std::shared_ptr<dai::NNData> nnData,
                std::shared_ptr<dai::ImgDetections> outDetections,
                DetectionParserProperties properties,
                std::shared_ptr<spdlog::async_logger> logger);

std::vector<std::string> getSortedDetectionLayerNames(std::shared_ptr<dai::NNData> nnData, std::string searchTerm, std::vector<std::string> outputNames);

float YoloIntersectionOverUnion(const DetectionCandidate& box1, const DetectionCandidate& box2);

bool isTensorOrderValid(dai::TensorInfo& tensorInfo, DetectionParserProperties properties, std::shared_ptr<spdlog::async_logger> logger);

void createImgDetections(std::vector<DetectionCandidate>& detectionCandidates,
                         std::vector<int> keepIndices,
                         std::shared_ptr<dai::ImgDetections> outDetections,
                         std::shared_ptr<spdlog::async_logger> logger);

std::vector<DetectionCandidate> nonMaximumSuppression(std::vector<DetectionCandidate>& detectionCandidates, float iouThr);

void createImgDetections(const std::vector<DetectionCandidate>& detectionCandidates,
                         std::shared_ptr<dai::ImgDetections> outDetections,
                         unsigned int width,
                         unsigned int height);

void segmentationDecode(std::shared_ptr<dai::NNData> nnData,
                        std::vector<DetectionCandidate>& detectionCandidates,
                        std::shared_ptr<dai::ImgDetections> outDetections,
                        DetectionParserProperties properties,
                        std::shared_ptr<spdlog::async_logger> logger);

void keypointDecode(std::shared_ptr<dai::NNData> nnData,
                    std::vector<DetectionCandidate>& detectionCandidates,
                    std::shared_ptr<dai::ImgDetections> outDetections,
                    DetectionParserProperties properties,
                    std::shared_ptr<spdlog::async_logger> logger);

}  // namespace DetectionParserUtils
}  // namespace utilities
}  // namespace dai