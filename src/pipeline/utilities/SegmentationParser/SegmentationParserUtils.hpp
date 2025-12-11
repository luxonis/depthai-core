#pragma once
#include <spdlog/async_logger.h>

#include <optional>

#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"
#include "pipeline/utilities/NNDataViewer.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

void parseSegmentationMask(const dai::NNData& nnData,
                           dai::TensorInfo& tensorInfo,
                           std::vector<uint8_t>& outputMask,
                           dai::SegmentationParserConfig& config,
                           std::shared_ptr<spdlog::async_logger>& logger);

// void parseSingleChannelSegmentationMask(const dai::NNData& nnData,
//                                         dai::SegmentationMask& outSegmentationMask,
//                                         dai::SegmentationParserConfig& config,
//                                         std::shared_ptr<spdlog::async_logger>& logger);

template <typename T>
void thresholdAndArgmaxTensor(std::vector<uint8_t>& dst, dai::NNDataViewer&, float threshold, int stepSize = 1);

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai