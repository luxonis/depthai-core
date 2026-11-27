#pragma once
#include <spdlog/async_logger.h>

#include <cstdint>

#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

void computeSegmentationMask(dai::SegmentationMask& outputMask,  // backgroundClass: class 0 is background and ignored in argmax
                             const dai::NNData& nnData,
                             const dai::TensorInfo& tensorInfo,
                             const dai::SegmentationParserConfig& config,
                             const bool backgroundClass,
                             std::shared_ptr<spdlog::async_logger>& logger);

template <typename T, bool backgroundClass>
void tensorArgmax(span<std::uint8_t> dstData,
                  span<const uint8_t> tensorSpan,
                  dai::TensorInfo& tensorInfo,
                  dai::SegmentationParserConfig& config,
                  std::shared_ptr<spdlog::async_logger>& logger);

template <bool backgroundClass>  // Separate from tensorArgmax to avoid template instantiation issues; NCHW (RVC2 FP16)
void tensorArgmaxFP16(span<std::uint8_t> dst,
                      span<const uint8_t> tensorSpan,
                      dai::TensorInfo& tensorInfo,
                      dai::SegmentationParserConfig& config,
                      std::shared_ptr<spdlog::async_logger>& logger);

void copySingleLayerMaskData(dai::SegmentationMask& outputMask,
                             dai::NNData& nnData,
                             dai::TensorInfo& tensorInfo,
                             dai::SegmentationParserConfig& config,
                             std::shared_ptr<spdlog::async_logger>& logger);

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai
