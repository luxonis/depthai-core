#pragma once
#include <spdlog/async_logger.h>

#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

void parseSegmentationMask(dai::NNData& nnData,
                           dai::TensorInfo& tensorInfo,
                           std::vector<uint8_t>& outputMask,
                           dai::SegmentationParserConfig& config,
                           std::shared_ptr<spdlog::async_logger>& logger);

template <typename T>
void thresholdAndArgmaxTensor(dai::SegmentationMask& dst,
                              dai::NNData& nnData,
                              dai::TensorInfo& tensorInfo,
                              dai::SegmentationParserConfig& config,
                              std::shared_ptr<spdlog::async_logger>& logger);

void thresholdAndArgmaxFp16Tensor(dai::SegmentationMask& dst, dai::NNData& nnData, dai::TensorInfo& tensorInfo, dai::SegmentationParserConfig& config);

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai