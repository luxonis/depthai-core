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

/**
 * @brief Compute segmentation mask from NNData tensor using the provided TensorInfo and SegmentationParserConfig.
 * @param outputMask SegmentationMask object to store the computed mask.
 * @param nnData NNData object containing the tensor data.
 * @param tensorInfo TensorInfo object describing the tensor layout and data type.
 * @param config SegmentationParserConfig object containing parsing parameters like confidenceThreshold and stepSize.
 * @param backgroundClass If true, the first class (index 0) is considered as background and ignored during calculating argmax.
 * @param logger Logger
 */
void computeSegmentationMask(dai::SegmentationMask& outputMask,
                             dai::NNData& nnData,
                             dai::TensorInfo& tensorInfo,
                             dai::SegmentationParserConfig& config,
                             bool backgroundClass,
                             std::shared_ptr<spdlog::async_logger>& logger);

/**
 * @brief get the class index with the highest thresholded confidenceThreshold for each pixel in the tensor. Function is optimized for NHWC and NCHW tensor
 * formats.
 * @tparam T Data type of the tensor (e.g., float, int8_t, uint8_t, ...)
 * @param dstData Output segmentation mask data (uint8_t)
 * @param tensorSpan Input tensor data
 * @param tensorInfo Information about the tensor (shape, data type, layout, etc.)
 * @param config SegmentationParserConfig containing parameters like confidenceThreshold and stepSize
 * @param logger Logger
 */
template <typename T, bool backgroundClass>
void tensorArgmax(span<std::uint8_t> dstData,
                  span<const uint8_t> tensorSpan,
                  dai::TensorInfo& tensorInfo,
                  dai::SegmentationParserConfig& config,
                  std::shared_ptr<spdlog::async_logger>& logger);

/**
 * @brief Specialized function for FP16 data type to avoid template instantiation issues. Optimized for NCHW tensor format, as FP16 is associated with RVC2
 * outputs.
 * @param dst Output segmentation mask data (uint8_t)
 * @param tensorSpan Input tensor data
 * @param tensorInfo Information about the tensor (shape, data type, layout, etc.)
 * @param config SegmentationParserConfig containing parameters like confidenceThreshold and stepSize
 * @param logger Logger
 */
template <bool backgroundClass>
void tensorArgmaxFP16(span<std::uint8_t> dst,
                      span<const uint8_t> tensorSpan,
                      dai::TensorInfo& tensorInfo,
                      dai::SegmentationParserConfig& config,
                      std::shared_ptr<spdlog::async_logger>& logger);

/**
 * @brief Copy segmentation mask from NNData tensor to SegmentationMask object. This is used when the segmentation classes are already processed (argmax) in the
 * NN.
 * @param outputMask SegmentationMask object to store the copied mask.
 * @param nnData NNData object containing the tensor data.
 * @param tensorInfo TensorInfo object describing the tensor layout and data type.
 * @param config SegmentationParserConfig object (not used in this function but kept for consistency).
 * @param logger Logger
 */
void copySingleLayerMaskData(dai::SegmentationMask& outputMask,
                             dai::NNData& nnData,
                             dai::TensorInfo& tensorInfo,
                             dai::SegmentationParserConfig& config,
                             std::shared_ptr<spdlog::async_logger>& logger);

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai
