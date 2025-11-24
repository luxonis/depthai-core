#include "DetectionParserUtils.hpp"

#include <spdlog/async_logger.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
#include <xtensor/core/xtensor_forward.hpp>

#include "DetectionParserUtils.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/properties/DetectionParserProperties.hpp"
#include "pipeline/utilities/NNDataViewer.hpp"

namespace dai {
namespace utilities {
namespace DetectionParserUtils {

// yolo v6 r1 - anchor free
void decodeR1AF(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger) {
    auto layerNames = utilities::DetectionParserUtils::getSortedDetectionLayerNames(nnData, "yolo", properties.parser.outputNamesToUse);

    const std::vector<int> strides = properties.parser.strides;
    if(strides.size() != layerNames.size()) {
        std::string errorMsg = fmt::format(
            "Number of strides does not match number of output layers. Strides size: {}, output layers size: {}", strides.size(), layerNames.size());
        throw std::runtime_error(errorMsg);
    }
    const float confidenceThr = properties.parser.confidenceThreshold;
    const float iouThr = properties.parser.iouThreshold;
    const int numClasses = properties.parser.classes;
    int inputWidth;
    int inputHeight;
    std::tie(inputWidth, inputHeight) = nnData.transformation->getSize();

    if(inputWidth <= 0 || inputHeight <= 0) {
        throw std::runtime_error("Invalid input dimensions retrieved from NNData transformation.");
    }
    std::vector<DetectionCandidate> detectionCandidates;
    detectionCandidates.reserve(defaultMaxDetectionsPerFrame);

    for(int strideIdx = 0; strideIdx < static_cast<int>(layerNames.size()); ++strideIdx) {
        std::string layerName = layerNames[strideIdx];
        auto tensorInfo = nnData.getTensorInfo(layerName);
        if(!tensorInfo) {
            std::string errorMsg = fmt::format("Tensor info for layer {} is null", layerName);
            throw std::runtime_error(errorMsg);
        }

        if(!isTensorOrderValid(*tensorInfo, properties, logger)) {
            logger->error("Tensor order for layer {} is invalid, skipping this layer", layerName);
            continue;
        }

        int layerHeight = tensorInfo->getHeight();
        int layerWidth = tensorInfo->getWidth();
        NNDataViewer outputData = NNDataViewer(*tensorInfo, nnData.data, logger);
        if(!outputData.build()) {
            std::string errorMsg = fmt::format("Failed to build NNDataViewer for layer {}", layerName);
            throw std::runtime_error(errorMsg);
        }

        for(int row = 0; row < layerHeight; ++row) {
            for(int col = 0; col < layerWidth; ++col) {
                const float objectnessScore = outputData.get(4, row, col);
                if(objectnessScore < confidenceThr) {
                    continue;
                }

                int bestC = 0;
                float bestConf = 0.0f;
                for(int c = 0; c < numClasses; ++c) {
                    float candidateProb = outputData.get(c + 5, row, col);
                    if(candidateProb > bestConf) {
                        bestConf = candidateProb;
                        bestC = c;
                    }
                }
                if(bestConf * objectnessScore < confidenceThr) {
                    continue;
                }

                float cx = outputData.get(0, row, col);
                float cy = outputData.get(1, row, col);
                float w = outputData.get(2, row, col);
                float h = outputData.get(3, row, col);

                float xmin = cx - w * 0.5f;
                float ymin = cy - h * 0.5f;
                float xmax = cx + w * 0.5f;
                float ymax = cy + h * 0.5f;

                xmin = std::max(0.0f, std::min(xmin, float(inputWidth)));
                ymin = std::max(0.0f, std::min(ymin, float(inputHeight)));
                xmax = std::max(0.0f, std::min(xmax, float(inputWidth)));
                ymax = std::max(0.0f, std::min(ymax, float(inputHeight)));

                if(xmax <= xmin || ymax <= ymin) {
                    logger->info("Invalid bbox parameters. Either xmax <= xmin or ymax <= ymin. Skipping detection.");
                    logger->debug(
                        "Skipping invalid bbox: layer='{}', "
                        "raw(cx,cy,w,h)=({:.2f},{:.2f},{:.2f},{:.2f}) "
                        "clamped(xmin,ymin,xmax,ymax)=({:.2f},{:.2f},{:.2f},{:.2f}).",
                        layerName,
                        cx,
                        cy,
                        w,
                        h,
                        xmin,
                        ymin,
                        xmax,
                        ymax);
                    continue;
                }
                DetectionCandidate candidate = DetectionCandidate{xmin, ymin, xmax, ymax, bestConf * objectnessScore, bestC, strideIdx, row, col, std::nullopt};

                detectionCandidates.emplace_back(std::move(candidate));
            }
        }
    }

    std::vector<DetectionCandidate> keepCandidates = nonMaximumSuppression(detectionCandidates, iouThr);
    if(keepCandidates.size() == 0) {
        logger->trace("No detections after NMS, skipping overlay.");
        return;
    }
    if(!properties.parser.classNames->empty()) {
        for(auto& candidate : keepCandidates) {
            candidate.labelName = (*properties.parser.classNames)[candidate.label];
        }
    }

    createImgDetections(keepCandidates, outDetections, inputWidth, inputHeight);

    if(properties.parser.decodeSegmentation) {
        logger->trace("Segmentation decoding.");
        segmentationDecode(nnData, keepCandidates, outDetections, properties, logger);
    }

    if(properties.parser.decodeKeypoints) {
        logger->trace("Keypoints decoding.");
        keypointDecode(nnData, keepCandidates, outDetections, properties, logger);
    }
}

/*
Decode anchor based yolo v3 and v3-Tiny
*/
void decodeV3AB(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger) {
    auto layerNames = getSortedDetectionLayerNames(nnData, "yolo", properties.parser.outputNamesToUse);
    auto sigmoid = [](float x) -> float { return 1.f / (1.f + std::exp(-x)); };

    const std::vector<int> strides = properties.parser.strides;
    if(strides.size() != layerNames.size()) {
        std::string errorMsg = fmt::format(
            "Number of strides does not match number of output layers. Strides size: {}, output layers size: {}", strides.size(), layerNames.size());
        throw std::runtime_error(errorMsg);
    }

    const float confidenceThr = properties.parser.confidenceThreshold;
    const float iouThr = properties.parser.iouThreshold;
    const int numClasses = properties.parser.classes;
    int inputWidth;
    int inputHeight;
    std::tie(inputWidth, inputHeight) = nnData.transformation->getSize();
    if(inputWidth <= 0 || inputHeight <= 0) {
        throw std::runtime_error("Invalid input dimensions retrieved from NNData transformation.");
    }

    if(properties.parser.anchorsV2.size() != layerNames.size()) {
        logger->error("Number of anchor sets does not match number of output layers. Anchor sets size: {}, output layers size: {}",
                      properties.parser.anchorsV2.size(),
                      layerNames.size());
        return;
    }

    std::vector<DetectionCandidate> detectionCandidates;
    detectionCandidates.reserve(defaultMaxDetectionsPerFrame);

    for(int strideIdx = 0; strideIdx < static_cast<int>(layerNames.size()); ++strideIdx) {
        std::string layerName = layerNames[strideIdx];
        int stride = strides[strideIdx];
        auto tensorInfo = nnData.getTensorInfo(layerName);
        if(!tensorInfo) {
            std::string errorMsg = fmt::format("Tensor info for layer {} is null", layerName);
            throw std::runtime_error(errorMsg);
        }

        if(!isTensorOrderValid(*tensorInfo, properties, logger)) {
            logger->error("Tensor order for layer {} is invalid, skipping this layer", layerName);
            continue;
        }

        int layerHeight = tensorInfo->getHeight();
        int layerWidth = tensorInfo->getWidth();
        int layerChannels = tensorInfo->getChannels();

        NNDataViewer outputData = NNDataViewer(*tensorInfo, nnData.data, logger);
        if(!outputData.build()) {
            std::string errorMsg = fmt::format("Failed to build NNDataViewer for layer {}", layerName);
            throw std::runtime_error(errorMsg);
        }
        std::vector<std::vector<float>>& anchors = properties.parser.anchorsV2[strideIdx];
        int numAnchors = anchors.size();
        int block = 5 + numClasses;
        int expectedC = numAnchors * block;

        if(layerChannels != expectedC) {
            std::string errorMsg = fmt::format("Layer {} channels mismatch. Expected {}, got {}", layerName, expectedC, layerChannels);
            throw std::runtime_error(errorMsg);
        }

        for(int row = 0; row < layerHeight; ++row) {
            for(int col = 0; col < layerWidth; ++col) {
                for(int a = 0; a < numAnchors; ++a) {
                    const int ch0 = a * block;
                    const float tx = sigmoid(outputData.get(ch0 + 0, row, col));
                    const float ty = sigmoid(outputData.get(ch0 + 1, row, col));
                    const float tw = outputData.get(ch0 + 2, row, col);
                    const float th = outputData.get(ch0 + 3, row, col);
                    const float obj = sigmoid(outputData.get(ch0 + 4, row, col));
                    if(obj < confidenceThr) continue;

                    int bestC = 0;
                    float clsLogit = 0.0f;
                    for(int c = 0; c < numClasses; ++c) {
                        const float candidateLogit = outputData.get(ch0 + 5 + c, row, col);
                        if(candidateLogit > clsLogit) {
                            clsLogit = candidateLogit;
                            bestC = c;
                        }
                    }
                    const float conf = obj * sigmoid(clsLogit);
                    if(conf < confidenceThr) continue;

                    // YOLOv3 decode
                    const float cx = (static_cast<float>(col) + tx) * static_cast<float>(stride);
                    const float cy = (static_cast<float>(row) + ty) * static_cast<float>(stride);
                    const float w_exp = std::exp(tw);
                    const float h_exp = std::exp(th);
                    const float w = w_exp * anchors[a][0];
                    const float h = h_exp * anchors[a][1];

                    float xmin = cx - 0.5f * w;
                    float ymin = cy - 0.5f * h;
                    float xmax = cx + 0.5f * w;
                    float ymax = cy + 0.5f * h;

                    xmin = std::max(0.0f, std::min(xmin, float(inputWidth)));
                    ymin = std::max(0.0f, std::min(ymin, float(inputHeight)));
                    xmax = std::max(0.0f, std::min(xmax, float(inputWidth)));
                    ymax = std::max(0.0f, std::min(ymax, float(inputHeight)));

                    if(xmax <= xmin || ymax <= ymin) {
                        logger->info("Invalid box with xmax <= xmin or ymax <= ymin, skipping");
                        continue;
                    }

                    DetectionCandidate candidate = DetectionCandidate{xmin, ymin, xmax, ymax, conf, bestC, strideIdx, row, col, std::nullopt};

                    detectionCandidates.emplace_back(std::move(candidate));
                }
            }
        }
    }

    std::vector<DetectionCandidate> keepCandidates = nonMaximumSuppression(detectionCandidates, iouThr);
    if(keepCandidates.size() == 0) {
        logger->trace("No detections after NMS, skipping overlay.");
        return;
    }

    if(!properties.parser.classNames->empty()) {
        for(auto& candidate : keepCandidates) {
            candidate.labelName = (*properties.parser.classNames)[candidate.label];
        }
    }

    createImgDetections(keepCandidates, outDetections, inputWidth, inputHeight);

    if(properties.parser.decodeSegmentation) {
        logger->trace("Segmentation decoding.");
        segmentationDecode(nnData, keepCandidates, outDetections, properties, logger);
    }

    if(properties.parser.decodeKeypoints) {
        logger->trace("Keypoints decoding.");
        keypointDecode(nnData, keepCandidates, outDetections, properties, logger);
    }

    //
}

/*
Decode anchor based networks, e.g., yolo v5, v7, P
*/
void decodeV5AB(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger) {
    auto layerNames = getSortedDetectionLayerNames(nnData, "yolo", properties.parser.outputNamesToUse);

    const std::vector<int> strides = properties.parser.strides;
    if(strides.size() != layerNames.size()) {
        std::string errorMsg = fmt::format(
            "Number of strides does not match number of output layers. Strides size: {}, output layers size: {}", strides.size(), layerNames.size());
        throw std::runtime_error(errorMsg);
    }

    const float confidenceThr = properties.parser.confidenceThreshold;
    const float iouThr = properties.parser.iouThreshold;
    const int numClasses = properties.parser.classes;
    int inputWidth;
    int inputHeight;
    std::tie(inputWidth, inputHeight) = nnData.transformation->getSize();

    if(inputWidth <= 0 || inputHeight <= 0) {
        throw std::runtime_error("Invalid input dimensions retrieved from NNData transformation.");
    }

    if(properties.parser.anchorsV2.size() != layerNames.size()) {
        logger->error("Number of anchor sets does not match number of output layers. Anchor sets size: {}, output layers size: {}",
                      properties.parser.anchorsV2.size(),
                      layerNames.size());
        return;
    }

    std::vector<DetectionCandidate> detectionCandidates;
    detectionCandidates.reserve(defaultMaxDetectionsPerFrame);

    for(int strideIdx = 0; strideIdx < static_cast<int>(layerNames.size()); ++strideIdx) {
        std::string layerName = layerNames[strideIdx];
        int stride = strides[strideIdx];
        auto tensorInfo = nnData.getTensorInfo(layerName);
        if(!tensorInfo) {
            std::string errorMsg = fmt::format("Tensor info for layer {} is null", layerName);
            throw std::runtime_error(errorMsg);
        }

        if(!isTensorOrderValid(*tensorInfo, properties, logger)) {
            logger->error("Tensor order for layer {} is invalid, skipping this layer", layerName);
            continue;
        }

        int layerHeight = tensorInfo->getHeight();
        int layerWidth = tensorInfo->getWidth();
        int layerChannels = tensorInfo->getChannels();

        NNDataViewer outputData = NNDataViewer(*tensorInfo, nnData.data, logger);
        if(!outputData.build()) {
            std::string errorMsg = fmt::format("Failed to build NNDataViewer for layer {}", layerName);
            throw std::runtime_error(errorMsg);
        }
        std::vector<std::vector<float>>& anchors = properties.parser.anchorsV2[strideIdx];
        int numAnchors = anchors.size();
        int block = 5 + numClasses;
        int expectedC = numAnchors * block;

        if(layerChannels != expectedC) {
            logger->error("Layer {} channels mismatch. Expected {}, got {}", layerName, expectedC, layerChannels);
            return;
        }

        for(int row = 0; row < layerHeight; ++row) {
            for(int col = 0; col < layerWidth; ++col) {
                for(int a = 0; a < numAnchors; ++a) {
                    const int ch0 = a * block;

                    const float tx = outputData.get(ch0 + 0, row, col);
                    const float ty = outputData.get(ch0 + 1, row, col);
                    const float tw = outputData.get(ch0 + 2, row, col);
                    const float th = outputData.get(ch0 + 3, row, col);
                    const float obj = outputData.get(ch0 + 4, row, col);
                    if(obj < confidenceThr) continue;

                    int bestC = 0;
                    float bestConf = 0.0f;
                    for(int c = 0; c < numClasses; ++c) {
                        const float candidateProb = outputData.get(ch0 + 5 + c, row, col);
                        if(candidateProb > bestConf) {
                            bestConf = candidateProb;
                            bestC = c;
                        }
                    }
                    const float conf = obj * bestConf;
                    if(conf < confidenceThr) continue;

                    // YOLOv5 decode
                    const float cx = ((tx * 2.0f - 0.5f) + static_cast<float>(col)) * static_cast<float>(stride);
                    const float cy = ((ty * 2.0f - 0.5f) + static_cast<float>(row)) * static_cast<float>(stride);

                    const float w = tw * tw * 4.0f * anchors[a][0];
                    const float h = th * th * 4.0f * anchors[a][1];

                    float xmin = cx - 0.5f * w;
                    float ymin = cy - 0.5f * h;
                    float xmax = cx + 0.5f * w;
                    float ymax = cy + 0.5f * h;

                    xmin = std::max(0.0f, std::min(xmin, float(inputWidth)));
                    ymin = std::max(0.0f, std::min(ymin, float(inputHeight)));
                    xmax = std::max(0.0f, std::min(xmax, float(inputWidth)));
                    ymax = std::max(0.0f, std::min(ymax, float(inputHeight)));

                    if(xmax <= xmin || ymax <= ymin) continue;
                    DetectionCandidate candidate = DetectionCandidate{xmin, ymin, xmax, ymax, conf, bestC, strideIdx, row, col, std::nullopt};

                    detectionCandidates.emplace_back(std::move(candidate));
                }
            }
        }
    }

    std::vector<DetectionCandidate> keepCandidates = nonMaximumSuppression(detectionCandidates, iouThr);
    if(keepCandidates.size() == 0) {
        logger->trace("No detections after NMS, skipping overlay.");
        return;
    }

    if(!properties.parser.classNames->empty()) {
        for(auto& candidate : keepCandidates) {
            candidate.labelName = (*properties.parser.classNames)[candidate.label];
        }
    }

    createImgDetections(keepCandidates, outDetections, inputWidth, inputHeight);

    if(properties.parser.decodeSegmentation) {
        logger->trace("Segmentation decoding.");
        segmentationDecode(nnData, keepCandidates, outDetections, properties, logger);
    }

    if(properties.parser.decodeKeypoints) {
        logger->trace("Keypoints decoding.");
        keypointDecode(nnData, keepCandidates, outDetections, properties, logger);
    }
}

/*
Decode TLBR (top left bottom right) style networks, e.g., yolo v6r2, v8, v10, v11
*/
void decodeTLBR(const dai::NNData& nnData,
                dai::ImgDetections& outDetections,
                DetectionParserProperties& properties,
                std::shared_ptr<spdlog::async_logger>& logger) {
    auto layerNames = DetectionParserUtils::getSortedDetectionLayerNames(nnData, "yolo", properties.parser.outputNamesToUse);

    const std::vector<int> strides = properties.parser.strides;
    if(strides.size() != layerNames.size()) {
        std::string errorMsg = fmt::format(
            "Number of strides does not match number of output layers. Strides size: {}, output layers size: {}", strides.size(), layerNames.size());
        throw std::runtime_error(errorMsg);
    }
    const float confidenceThr = properties.parser.confidenceThreshold;
    const float iouThr = properties.parser.iouThreshold;
    const int numClasses = properties.parser.classes;
    int inputWidth;
    int inputHeight;
    std::tie(inputWidth, inputHeight) = nnData.transformation->getSize();

    if(inputWidth <= 0 || inputHeight <= 0) {
        throw std::runtime_error("Invalid input dimensions retrieved from NNData transformation.");
    }

    std::vector<DetectionCandidate> detectionCandidates;
    detectionCandidates.reserve(defaultMaxDetectionsPerFrame);

    for(int strideIdx = 0; strideIdx < static_cast<int>(layerNames.size()); ++strideIdx) {
        std::string layerName = layerNames[strideIdx];
        int stride = strides[strideIdx];
        auto tensorInfo = nnData.getTensorInfo(layerName);
        if(!tensorInfo) {
            std::string errorMsg = fmt::format("Tensor info for layer {} is null", layerName);
            throw std::runtime_error(errorMsg);
        }

        if(!isTensorOrderValid(*tensorInfo, properties, logger)) {
            logger->error("Tensor order for layer {} is invalid, skipping this layer", layerName);
            continue;
        }

        int layerHeight = tensorInfo->getHeight();
        int layerWidth = tensorInfo->getWidth();
        NNDataViewer outputData = NNDataViewer(*tensorInfo, nnData.data, logger);
        if(!outputData.build()) {
            std::string errorMsg = fmt::format("Failed to build NNDataViewer for layer {}", layerName);
            throw std::runtime_error(errorMsg);
        }

        for(int row = 0; row < layerHeight; ++row) {
            for(int col = 0; col < layerWidth; ++col) {
                const float score = outputData.get(4, row, col);
                if(score < confidenceThr) {
                    continue;
                }

                int bestC = 0;
                float bestConf = 0.0f;
                for(int c = 0; c < numClasses; ++c) {
                    float candidateProb = outputData.get(c + 5, row, col);
                    if(candidateProb > bestConf) {
                        bestConf = candidateProb;
                        bestC = c;
                    }
                }
                float xmin = (col - outputData.get(0, row, col) + 0.5f) * stride;
                float ymin = (row - outputData.get(1, row, col) + 0.5f) * stride;
                float xmax = (col + outputData.get(2, row, col) + 0.5f) * stride;
                float ymax = (row + outputData.get(3, row, col) + 0.5f) * stride;

                if(bestConf < confidenceThr) {
                    continue;
                }

                xmin = std::max(0.0f, std::min(xmin, float(inputWidth)));
                ymin = std::max(0.0f, std::min(ymin, float(inputHeight)));
                xmax = std::max(0.0f, std::min(xmax, float(inputWidth)));
                ymax = std::max(0.0f, std::min(ymax, float(inputHeight)));

                if(xmax <= xmin || ymax <= ymin) {
                    logger->info("Invalid box with xmax <= xmin or ymax <= ymin, skipping");
                    continue;
                }

                DetectionCandidate candidate = DetectionCandidate{xmin, ymin, xmax, ymax, bestConf, bestC, strideIdx, row, col, std::nullopt};

                detectionCandidates.emplace_back(std::move(candidate));
            }
        }
    }

    std::vector<DetectionCandidate> keepCandidates = nonMaximumSuppression(detectionCandidates, iouThr);
    if(keepCandidates.size() == 0) {
        logger->trace("No detections after NMS, skipping overlay.");
        return;
    }

    if(!properties.parser.classNames->empty()) {
        for(auto& candidate : keepCandidates) {
            candidate.labelName = (*properties.parser.classNames)[candidate.label];
        }
    }

    createImgDetections(keepCandidates, outDetections, inputWidth, inputHeight);

    if(properties.parser.decodeSegmentation) {
        logger->trace("Segmentation decoding.");
        segmentationDecode(nnData, keepCandidates, outDetections, properties, logger);
    }

    if(properties.parser.decodeKeypoints) {
        logger->trace("Keypoints decoding.");
        keypointDecode(nnData, keepCandidates, outDetections, properties, logger);
    }
}

bool isTensorOrderValid(dai::TensorInfo& tensorInfo, DetectionParserProperties properties, std::shared_ptr<spdlog::async_logger>& logger) {
    int anchorMultiplier = properties.parser.anchorsV2.empty() ? 1 : static_cast<int>(properties.parser.anchorsV2.size());
    int channelSize = anchorMultiplier * (properties.parser.classes + properties.parser.coordinates + 1);

    auto checkAndFixOrder = [&](int channelDimIndex, int alternativeDimIndex, dai::TensorInfo::StorageOrder alternativeOrder) -> bool {
        // Check that the dims size is big enough
        if(static_cast<int>(tensorInfo.dims.size()) <= channelDimIndex || static_cast<int>(tensorInfo.dims.size()) <= alternativeDimIndex) {
            logger->error("Invalid tensor dims size. Skipping.");
            return false;
        }

        if(tensorInfo.dims[channelDimIndex] != uint32_t(channelSize)) {
            // Check if the channel size would match the alternative storage order
            if(tensorInfo.dims[alternativeDimIndex] == uint32_t(channelSize)) {
                logger->trace("Invalid channel size for the tensor. Expected {}, got {}, switching", channelSize, tensorInfo.dims[channelDimIndex]);
                tensorInfo.order = alternativeOrder;
            } else {
                logger->error("Invalid channel size for the tensor. Expected {}, got {}. Skipping.", channelSize, tensorInfo.dims[channelDimIndex]);
                return false;
            }
        }
        return true;
    };

    switch(tensorInfo.order) {
        case dai::TensorInfo::StorageOrder::CHW:
            if(!checkAndFixOrder(0, 2, dai::TensorInfo::StorageOrder::HWC)) return false;
            break;
        case dai::TensorInfo::StorageOrder::HWC:
            if(!checkAndFixOrder(2, 0, dai::TensorInfo::StorageOrder::CHW)) return false;
            break;
        case dai::TensorInfo::StorageOrder::NCHW:
            if(!checkAndFixOrder(1, 3, dai::TensorInfo::StorageOrder::NHWC)) return false;
            break;
        case dai::TensorInfo::StorageOrder::NHWC:
            if(!checkAndFixOrder(3, 1, dai::TensorInfo::StorageOrder::NCHW)) return false;
            break;
        case dai::TensorInfo::StorageOrder::NHCW:
        case dai::TensorInfo::StorageOrder::WHC:
        case dai::TensorInfo::StorageOrder::WCH:
        case dai::TensorInfo::StorageOrder::HCW:
        case dai::TensorInfo::StorageOrder::CWH:
        case dai::TensorInfo::StorageOrder::NC:
        case dai::TensorInfo::StorageOrder::CN:
        case dai::TensorInfo::StorageOrder::C:
        case dai::TensorInfo::StorageOrder::H:
        case dai::TensorInfo::StorageOrder::W:
        default:
            logger->error("Invalid storage order for the tensor. Skipping.");
            return false;
    }

    return true;
}

std::vector<std::string> getSortedDetectionLayerNames(const dai::NNData& nnData, std::string searchTerm, std::vector<std::string> outputNames) {
    if(outputNames.empty()) {
        outputNames = nnData.getAllLayerNames();
    }

    std::vector<std::string> layerNames;
    for(const auto& name : outputNames) {
        // if yolo in the name, push it to layerNames
        if(name.find(searchTerm) != std::string::npos) {
            layerNames.push_back(name);
        }
    }

    std::sort(layerNames.begin(), layerNames.end());
    return layerNames;
}

float YoloIntersectionOverUnion(const DetectionCandidate& box1, const DetectionCandidate& box2) {
    float width_of_overlap_area = fmin(box1.xmax, box2.xmax) - fmax(box1.xmin, box2.xmin);
    float height_of_overlap_area = fmin(box1.ymax, box2.ymax) - fmax(box1.ymin, box2.ymin);
    float area_of_overlap;
    if(width_of_overlap_area < 0 || height_of_overlap_area < 0)
        area_of_overlap = 0;
    else
        area_of_overlap = width_of_overlap_area * height_of_overlap_area;
    float box_1_area = (box1.ymax - box1.ymin) * (box1.xmax - box1.xmin);
    float box_2_area = (box2.ymax - box2.ymin) * (box2.xmax - box2.xmin);
    float area_of_union = box_1_area + box_2_area - area_of_overlap;
    return area_of_overlap / area_of_union;
}

std::vector<DetectionCandidate> nonMaximumSuppression(std::vector<DetectionCandidate>& detectionCandidates, float iouThr) {
    std::sort(
        detectionCandidates.begin(), detectionCandidates.end(), [](const DetectionCandidate& a, const DetectionCandidate& b) { return a.score > b.score; });

    std::vector<uint8_t> keep(detectionCandidates.size(), 1);
    std::vector<size_t> keepIndices;
    keepIndices.reserve(detectionCandidates.size());

    for(size_t i = 0; i < detectionCandidates.size(); ++i) {
        if(!keep[i]) continue;
        keepIndices.push_back(i);

        for(size_t j = i + 1; j < detectionCandidates.size(); ++j) {
            if(!keep[j]) continue;
            if(YoloIntersectionOverUnion(detectionCandidates[i], detectionCandidates[j]) >= iouThr) {
                keep[j] = 0;
            }
        }
    }

    std::vector<DetectionCandidate> keepCandidates;
    keepCandidates.reserve(keepIndices.size());
    for(size_t idx : keepIndices) keepCandidates.push_back(detectionCandidates[idx]);

    return keepCandidates;
}

void createImgDetections(const std::vector<DetectionCandidate>& detectionCandidates,
                         dai::ImgDetections& outDetections,
                         unsigned int width,
                         unsigned int height) {
    for(const auto& det : detectionCandidates) {
        dai::ImgDetection detection;
        dai::RotatedRect rotatedRect(dai::Rect(dai::Point2f(det.xmin, det.ymin), dai::Point2f(det.xmax, det.ymax)), 0.0f);
        detection.setBoundingBox(rotatedRect.normalize(width, height));
        detection.confidence = det.score;
        detection.label = det.label;
        if(det.labelName) {
            detection.labelName = *det.labelName;
        }
        outDetections.detections.push_back(std::move(detection));
    }
}

void segmentationDecode(const dai::NNData& nnData,
                        std::vector<DetectionCandidate>& detectionCandidates,
                        dai::ImgDetections& outDetections,
                        DetectionParserProperties properties,
                        std::shared_ptr<spdlog::async_logger>& logger) {
    std::pair<int, int> inputSize = nnData.transformation->getSize();
    int inputWidth = inputSize.first;
    int inputHeight = inputSize.second;

    logger->warn("printing");
    cv::Mat indexMask(inputHeight, inputWidth, CV_8U, cv::Scalar(255));

    auto maskLayerNames = DetectionParserUtils::getSortedDetectionLayerNames(nnData, "masks", std::vector<std::string>{});
    if(properties.parser.strides.size() != maskLayerNames.size()) {
        logger->error(
            "Number of strides does not match number of mask output layers. Strides size: {}, mask output layers size: {}. Skipping segmentation decoding.",
            properties.parser.strides.size(),
            maskLayerNames.size());
        return;
    }
    auto protoLayerNames = DetectionParserUtils::getSortedDetectionLayerNames(nnData, "proto", std::vector<std::string>{});
    if(protoLayerNames.size() == 0) {
        logger->error("Expecting proto output layer, found no layer with proto label. Skipping segmentation decoding.");
        return;
    }

    NNDataViewer protoValues = NNDataViewer(*nnData.getTensorInfo(protoLayerNames[0]), nnData.data, logger);
    if(!protoValues.build()) {
        logger->error("Failed to build NNDataViewer for proto layer {}. Skipping segmentation decoding.", protoLayerNames[0]);
        return;
    }

    TensorInfo protoInfo = *nnData.getTensorInfo(protoLayerNames[0]);
    int protoWidth = protoInfo.getWidth();
    int protoHeight = protoInfo.getHeight();
    int protoChannels = protoInfo.getChannels();
    if(protoWidth <= 0 || protoHeight <= 0 || protoChannels <= 0) {
        logger->error("Invalid proto tensor dimensions: channels {}, height {}, width {}.", protoChannels, protoHeight, protoWidth);
        return;
    }
    int protoWidthScaleFactor = inputWidth / protoWidth;
    int protoHeightScaleFactor = inputHeight / protoHeight;

    cv::Mat maskUp;
    cv::Mat maskLow(protoHeight, protoWidth, CV_32F);

    dai::NNData& nnDataNonConst = const_cast<dai::NNData&>(nnData);
    xt::xarray<float> protoData = nnDataNonConst.getTensor<float>(protoLayerNames[0], true);
    if(protoInfo.order != dai::TensorInfo::StorageOrder::NHWC) {
        logger->trace("Proto storage is not NHWC, changing order.");
        nnDataNonConst.changeStorageOrder(protoData, protoInfo.order, dai::TensorInfo::StorageOrder::NHWC);
    }
    Eigen::MatrixXf protoMatrix = Eigen::Map<Eigen::MatrixXf>(protoData.data(), protoChannels, protoHeight * protoWidth);

    Eigen::RowVectorXf coeffs(protoChannels);

    auto maskFromCoeffs = [logger, protoHeight, protoWidth, &maskLow](const Eigen::MatrixXf& protos2d, const Eigen::RowVectorXf& coeffs) -> void {
        if(protos2d.rows() != coeffs.size()) {
            throw std::runtime_error("Mask coefficients size does not match proto channels.");
        }

        Eigen::Map<Eigen::RowVectorXf> logits(maskLow.ptr<float>(), protoHeight * protoWidth);
        logits.noalias() = coeffs * protos2d;

        // no need to do sigmoid
        // logits = (1.0f / (1.0f + (-logits.array()).exp())).matrix();
    };

    std::map<int, NNDataViewer> maskValues;
    for(int strideIdx = 0; strideIdx < static_cast<int>(maskLayerNames.size()); ++strideIdx) {
        maskValues.try_emplace(strideIdx, *nnData.getTensorInfo(maskLayerNames[strideIdx]), nnData.data, logger);
        if(!maskValues.at(strideIdx).build()) {
            logger->error("Failed to build NNDataViewer for mask layer {}. Skipping segmentation decoding.", maskLayerNames[strideIdx]);
            return;
        }
    }

    for(size_t i = 0; i < detectionCandidates.size(); ++i) {  // loop over all detections
        const auto& c = detectionCandidates[i];
        const int detIdx = static_cast<int>(i);  // index in outDetections list

        NNDataViewer& mask = maskValues.at(c.headIndex);
        for(int ch = 0; ch < protoChannels; ++ch) {
            coeffs(ch) = mask.get(ch, c.rowIndex, c.columnIndex);
        }
        // TODO (aljaz) perform operations on ROI only instead of the full resolution
        // Eigen::MatrixXf roiMatrix = protoMatrix.block(0, y0 * protoWidth + x0, protoChannels, (y1 - y0) * (x1 - x0));

        maskFromCoeffs(protoMatrix, coeffs);

        int x0 = std::clamp(static_cast<int>(std::floor(c.xmin)), 0, inputWidth - 1);
        int y0 = std::clamp(static_cast<int>(std::floor(c.ymin)), 0, inputHeight - 1);
        int x1 = std::clamp(static_cast<int>(std::ceil(c.xmax)), 0, inputWidth);
        int y1 = std::clamp(static_cast<int>(std::ceil(c.ymax)), 0, inputHeight);

        if(x1 <= x0 || y1 <= y0) continue;
        const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);

        int protoX0 = x0 / protoWidthScaleFactor;
        int protoY0 = y0 / protoHeightScaleFactor;
        int protoX1 = x1 / protoWidthScaleFactor;
        int protoY1 = y1 / protoHeightScaleFactor;
        const cv::Rect protoROI(protoX0, protoY0, protoX1 - protoX0, protoY1 - protoY0);

        cv::Mat roiProb;
        cv::resize(maskLow(protoROI), roiProb, roi.size(), 0, 0, cv::INTER_LINEAR);

        // Threshold & paint only unassigned pixels
        cv::Mat roiBin;
        cv::compare(roiProb, 0.0, roiBin, cv::CMP_GT);
        cv::Mat roiOut = indexMask(roi);
        cv::Mat unassigned;
        cv::compare(roiOut, 255, unassigned, cv::CMP_EQ);
        cv::Mat paintMask;
        cv::bitwise_and(roiBin, unassigned, paintMask);

        const uint8_t value = static_cast<uint8_t>(std::min(detIdx, 254));
        roiOut.setTo(value, paintMask);
    }

    outDetections.setCvSegmentationMask(indexMask);
}

void keypointDecode(const dai::NNData& nnData,
                    std::vector<DetectionCandidate>& detectionCandidates,
                    dai::ImgDetections& outDetections,
                    DetectionParserProperties properties,
                    std::shared_ptr<spdlog::async_logger>& logger) {
    if(!properties.parser.nKeypoints) {
        logger->warn("Number of keypoints not set in properties.parser.nKeypoints. Skipping keypoints decoding.");
        return;
    }

    int inputWidth;
    int inputHeight;
    std::tie(inputWidth, inputHeight) = nnData.transformation->getSize();

    auto yoloLayerNames = DetectionParserUtils::getSortedDetectionLayerNames(nnData, "yolo", properties.parser.outputNamesToUse);
    std::vector<int> featureMapWidths;
    for(int i = 0; i < static_cast<int>(yoloLayerNames.size()); ++i) {
        auto tensorInfo = nnData.getTensorInfo(yoloLayerNames[i]);
        if(!tensorInfo) {
            logger->error("Tensor info for layer {} is null. Skipping keypoints decoding.", yoloLayerNames[i]);
            return;
        }
        featureMapWidths.push_back(tensorInfo->getWidth());
    }

    auto kptsLayerNames = DetectionParserUtils::getSortedDetectionLayerNames(nnData, "kpt_output", std::vector<std::string>{});
    if(properties.parser.strides.size() != kptsLayerNames.size()) {
        logger->error(
            "Number of strides does not match number of keypoints output layers. Strides size: {}, keypoints output layers size: {}. Skipping keypoints "
            "decoding.",
            properties.parser.strides.size(),
            kptsLayerNames.size());
        return;
    }

    // TODO (aljaz) move to a function
    std::map<int, NNDataViewer> keypointValues;
    for(int strideIdx = 0; strideIdx < static_cast<int>(kptsLayerNames.size()); ++strideIdx) {
        keypointValues.try_emplace(strideIdx, *nnData.getTensorInfo(kptsLayerNames[strideIdx]), nnData.data, logger);
        if(!keypointValues.at(strideIdx).build()) {
            logger->error("Failed to build NNDataViewer for keypoints layer {}. Skipping keypoints decoding.", kptsLayerNames[strideIdx]);
            return;
        }
    }

    if(outDetections.detections.size() != detectionCandidates.size()) {
        logger->error(
            "Number of detections in ImgDetections does not match number of detection candidates. ImgDetections size: {}, detection candidates size: {}. "
            "Skipping keypoints decoding.",
            outDetections.detections.size(),
            detectionCandidates.size());
        return;
    }

    for(size_t i = 0; i < detectionCandidates.size(); ++i) {  // loop over all detections
        const auto& c = detectionCandidates[i];
        int flattenedIndex = c.rowIndex * featureMapWidths[c.headIndex] + c.columnIndex;

        std::vector<dai::Keypoint> keypoints;
        keypoints.reserve(*properties.parser.nKeypoints);
        NNDataViewer keypointMask = keypointValues.at(c.headIndex);

        for(int k = 0; k < properties.parser.nKeypoints; ++k) {
            int base = 3 * k;

            // keypointValues tensor storage order HWC
            //  H == 0
            //  W == 51 == 17 * 3 (x, y, conf for each keypoint)
            //  C == flattened spatial dimensions of row x col of the feature map
            float x = std::clamp(keypointMask.get(flattenedIndex, 0, base + 0) / inputWidth, 0.0f, 1.0f);
            float y = std::clamp(keypointMask.get(flattenedIndex, 0, base + 1) / inputHeight, 0.0f, 1.0f);
            float conf = 1.f / (1.f + std::exp(-(keypointMask.get(flattenedIndex, 0, base + 2))));

            keypoints.push_back(dai::Keypoint{dai::Point2f(x, y), conf});
        }
        outDetections.detections[i].keypoints = KeypointsList(keypoints, properties.parser.keypointEdges);
    }
}

}  // namespace DetectionParserUtils
}  // namespace utilities
}  // namespace dai
