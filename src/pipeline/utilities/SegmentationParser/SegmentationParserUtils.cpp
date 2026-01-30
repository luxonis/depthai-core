#include "SegmentationParserUtils.hpp"

#include <fp16/fp16.h>
#include <spdlog/async_logger.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "pipeline/datatype/SegmentationParserConfig.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

constexpr uint8_t BACKGROUND_INDEX = 255;

template <typename T>
inline T loadT(const uint8_t* p) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "Segmentation type must be trivially copyable");
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

template <typename T>
const T* assumeAlignedPtr(const uint8_t* ptr) {
#if defined(__GNUC__) || defined(__clang__)
    return static_cast<const T*>(__builtin_assume_aligned(ptr, alignof(T)));
#elif defined(_MSC_VER)
    __assume((reinterpret_cast<uintptr_t>(ptr) & (alignof(T) - 1)) == 0);
    return reinterpret_cast<const T*>(ptr);
#else
    return reinterpret_cast<const T*>(ptr);
#endif
}

template <typename T>
T getQuantizedThreshold(const dai::SegmentationParserConfig& config, const dai::TensorInfo& tensorInfo) {
    if(config.confidenceThreshold < 0) {
        // Minimum possible value for int8 and uint8 quantizations
        return tensorInfo.dataType == dai::TensorInfo::DataType::I8 ? static_cast<T>(-128) : static_cast<T>(0);
    }
    return static_cast<T>((config.confidenceThreshold / tensorInfo.qpScale) + tensorInfo.qpZp);
}

/*
 * @brief Optimized argmax function for NHWC tensors with aligned and typed channel strides.
 * Data looks like [h1w1c1, h1w1c2, ..., h1w1cC, h1w2c1, h1w2c2, ..., h1w2cC, ..., hHwWcC]
 */
template <typename T, bool backgroundClass>
void argmaxNHWCVectorizedTensor(span<uint8_t> dst,
                                const span<const uint8_t> tensorBase,
                                dai::TensorInfo& tensorInfo,
                                const dai::SegmentationParserConfig& config) {
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideC = tensorInfo.getChannelStride() / sizeof(T);
    const size_t strideH = tensorInfo.getHeightStride() / sizeof(T);
    const size_t strideW = tensorInfo.getWidthStride() / sizeof(T);
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    const T* __restrict tensorPtr = assumeAlignedPtr<T>(tensorBase.data());

    int outY = 0;
    uint8_t bestClass = 255;
    T quantizedThreshold = getQuantizedThreshold<T>(config, tensorInfo);
    T bestVal = quantizedThreshold;
    const int cStart = backgroundClass ? 1 : 0;

    for(int h = 0; h < height; h += step, ++outY) {
        const size_t rowBase = strideH * static_cast<size_t>(h);
        const int rowOffset = outY * dstWidth;

        int outX = 0;
        for(int w = 0; w < width; w += step, ++outX) {
            const size_t pixelBase = rowBase + (strideW * static_cast<size_t>(w));

            bestClass = 255;
            if constexpr(backgroundClass) {
                bestVal = std::max(tensorPtr[pixelBase], quantizedThreshold);
            } else {
                bestVal = quantizedThreshold;
            }
            for(int c = cStart; c < channels; ++c) {
                const size_t idx = pixelBase + (strideC * static_cast<size_t>(c));
                T v = tensorPtr[idx];
                if(v > bestVal) {
                    bestVal = v;
                    bestClass = static_cast<uint8_t>(c);
                }
            }
            dst[rowOffset + outX] = bestClass;
        }
    }
}

/*
 * @brief Parse NHWC formatted tensor to compute segmentation mask.
 * Data looks like [h1w1c1, h1w1c2, ..., h1w1cC, h1w2c1, h1w2c2, ..., h1w2cC, ..., hHwWcC]
 */
template <typename T, bool backgroundClass>
void parseNHWCTensor(span<uint8_t> dst, const span<const uint8_t> tensorBase, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config) {
    const uint8_t* tensorPtr = tensorBase.data();
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideC = tensorInfo.getChannelStride();
    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    int outY = 0;
    uint8_t bestClass = 255;
    T quantizedThreshold = getQuantizedThreshold<T>(config, tensorInfo);
    T bestVal = quantizedThreshold;
    const int cStart = backgroundClass ? 1 : 0;

    for(int h = 0; h < height; h += step, ++outY) {
        const size_t rowBase = strideH * static_cast<size_t>(h);
        const int rowOffset = outY * dstWidth;
        int outX = 0;

        for(int w = 0; w < width; w += step, ++outX) {
            const size_t pixelBase = rowBase + (strideW * static_cast<size_t>(w));
            bestClass = 255;
            if constexpr(backgroundClass) {
                bestVal = std::max(loadT<T>(tensorPtr + pixelBase), quantizedThreshold);
            } else {
                bestVal = quantizedThreshold;
            }

            for(int c = cStart; c < channels; ++c) {
                const size_t idx = pixelBase + (strideC * static_cast<size_t>(c));
                T v = loadT<T>(tensorPtr + idx);
                if(v > bestVal) {
                    bestVal = v;
                    bestClass = static_cast<uint8_t>(c);
                }
            }
            dst[rowOffset + outX] = bestClass;
        }
    }
}

template <typename T, bool backgroundClass>
void fillBestVals(std::vector<T>& bestValues, const uint8_t* tensorPtr, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config) {
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    const T quantizedThreshold = getQuantizedThreshold<T>(config, tensorInfo);

    if constexpr(backgroundClass) {
        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const size_t rowBase = (strideH * static_cast<size_t>(h));
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const size_t idx = rowBase + (strideW * static_cast<size_t>(w));
                const T v0 = loadT<T>(tensorPtr + idx);
                bestValues[(outY * dstWidth) + outX] = std::max(v0, quantizedThreshold);
            }
        }
    } else {
        std::fill(bestValues.begin(), bestValues.end(), quantizedThreshold);
    }
}

/*
 * @brief Optimized argmax function over NCHW formatted tensor.
 * Data looks like [h1w1c1, h1w2c1, ..., hMwNc1, h1w1c2, h1w2c2, ..., hMwNcC]
 */

template <typename T, bool backgroundClass>
void parseNCHWTensor(span<uint8_t> dst, const span<const uint8_t> tensorBase, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config) {
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideC = tensorInfo.getChannelStride();
    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    static thread_local std::vector<T> bestValues;
    bestValues.resize(dst.size());
    const uint8_t* tensorPtr = tensorBase.data();
    fillBestVals<T, backgroundClass>(bestValues, tensorPtr, tensorInfo, config);

    const int cStart = backgroundClass ? 1 : 0;
    for(int c = cStart; c < channels; c++) {
        const size_t channelBase = strideC * static_cast<size_t>(c);
        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const size_t rowBase = channelBase + (strideH * static_cast<size_t>(h));
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const size_t idx = rowBase + (strideW * static_cast<size_t>(w));
                T v = loadT<T>(tensorPtr + idx);
                int coordinateIndex = (outY * dstWidth) + outX;
                if(v > bestValues[coordinateIndex]) {
                    bestValues[coordinateIndex] = v;
                    dst[coordinateIndex] = static_cast<uint8_t>(c);
                }
            }
        }
    }
}

template <typename T, bool backgroundClass>
void tensorArgmax(span<std::uint8_t> dstData,
                  const span<const uint8_t> tensorSpan,
                  dai::TensorInfo& tensorInfo,
                  dai::SegmentationParserConfig& config,
                  std::shared_ptr<spdlog::async_logger>& logger) {
    /*
     * Should be moved to GPU.
     */
    const bool vectorizable = (tensorInfo.getChannelStride() == static_cast<int>(sizeof(T)))
                              && (tensorInfo.getHeightStride() == tensorInfo.getWidthStride() * tensorInfo.getWidth())
                              && (tensorInfo.getWidthStride() == static_cast<size_t>(tensorInfo.getChannels()) * static_cast<int>(sizeof(T)));
    const bool isNHWC = (tensorInfo.order == dai::TensorInfo::StorageOrder::NHWC || tensorInfo.order == dai::TensorInfo::StorageOrder::HWC);
    if(vectorizable && isNHWC) {
        logger->trace("Using vectorized NHWC segmentation mask parsing.");
        argmaxNHWCVectorizedTensor<T, backgroundClass>(dstData, tensorSpan, tensorInfo, config);
    } else if(isNHWC) {
        logger->trace("Using NHWC segmentation mask parsing.");
        parseNHWCTensor<T, backgroundClass>(dstData, tensorSpan, tensorInfo, config);
    } else {
        logger->trace("Defaulting to NCHW/CHW segmentation mask parsing.");
        parseNCHWTensor<T, backgroundClass>(dstData, tensorSpan, tensorInfo, config);
    }
}

template <bool backgroundClass>
void fillFP16BestVals(std::vector<float>& bestValues, const uint8_t* tensorPtr, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config) {
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    const float thr = (config.confidenceThreshold < 0.0f) ? -std::numeric_limits<float>::infinity() : config.confidenceThreshold;

    if constexpr(backgroundClass) {
        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const size_t rowBase = (strideH * static_cast<size_t>(h));
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const size_t idx = rowBase + (strideW * static_cast<size_t>(w));
                float v = fp16_ieee_to_fp32_value(loadT<uint16_t>(tensorPtr + idx));
                bestValues[(outY * dstWidth) + outX] = std::max(v, thr);
            }
        }
    } else {
        std::fill(bestValues.begin(), bestValues.end(), thr);
    }
}

template <bool backgroundClass>
void tensorArgmaxFP16(span<std::uint8_t> dst,
                      const span<const uint8_t> tensorSpan,
                      dai::TensorInfo& tensorInfo,
                      dai::SegmentationParserConfig& config,
                      std::shared_ptr<spdlog::async_logger>& logger) {
    logger->trace("Using FP16 NCHW segmentation mask parsing.");
    const uint8_t* tensorBase = tensorSpan.data();
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const size_t strideC = tensorInfo.getChannelStride();
    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int step = static_cast<int>(config.stepSize);
    const int dstWidth = width / step;

    static thread_local std::vector<float> bestVals;
    bestVals.resize(dst.size());
    fillFP16BestVals<backgroundClass>(bestVals, tensorBase, tensorInfo, config);

    const int cStart = backgroundClass ? 1 : 0;
    for(int c = cStart; c < channels; c++) {
        const size_t channelBase = strideC * static_cast<size_t>(c);

        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const size_t rowBase = channelBase + (strideH * static_cast<size_t>(h));
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const size_t idx = rowBase + (strideW * static_cast<size_t>(w));
                float v = fp16_ieee_to_fp32_value(loadT<uint16_t>(tensorBase + idx));
                int coordinateIndex = (outY * dstWidth) + outX;
                if(v > bestVals[coordinateIndex]) {
                    bestVals[coordinateIndex] = v;
                    dst[coordinateIndex] = static_cast<uint8_t>(c);
                }
            }
        }
    }
}

void computeSegmentationMask(dai::SegmentationMask& outputMask,
                             dai::NNData& nnData,
                             dai::TensorInfo& tensorInfo,
                             dai::SegmentationParserConfig& config,
                             const bool backgroundClass,
                             std::shared_ptr<spdlog::async_logger>& logger) {
    span<std::uint8_t> dstData =
        outputMask.prepareMask(static_cast<size_t>(tensorInfo.getWidth() / config.stepSize), static_cast<size_t>(tensorInfo.getHeight() / config.stepSize));
    std::fill(dstData.begin(), dstData.end(), BACKGROUND_INDEX);

    auto nnMemory = nnData.data;
    const span<const uint8_t> nnBytes = nnMemory->getData();
    const span<const uint8_t> allTensors{nnBytes.data(), nnBytes.size()};
    const span<const uint8_t> tensorSpan = allTensors.subspan(tensorInfo.offset, tensorInfo.getTensorSize());

    switch((tensorInfo).dataType) {
        case dai::TensorInfo::DataType::I8:
            if(backgroundClass) {
                tensorArgmax<int8_t, true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmax<int8_t, false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        case dai::TensorInfo::DataType::U8F:
            if(backgroundClass) {
                tensorArgmax<uint8_t, true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmax<uint8_t, false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        case dai::TensorInfo::DataType::INT:
            if(backgroundClass) {
                tensorArgmax<int32_t, true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmax<int32_t, false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        case dai::TensorInfo::DataType::FP32:
            if(backgroundClass) {
                tensorArgmax<float, true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmax<float, false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        case dai::TensorInfo::DataType::FP16:
            if(backgroundClass) {
                tensorArgmaxFP16<true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmaxFP16<false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        case dai::TensorInfo::DataType::FP64:
            if(backgroundClass) {
                tensorArgmax<double_t, true>(dstData, tensorSpan, tensorInfo, config, logger);
            } else {
                tensorArgmax<double_t, false>(dstData, tensorSpan, tensorInfo, config, logger);
            }
            break;
        default:
            logger->error("Unsupported data type for segmentation parsing: {}", static_cast<int>(tensorInfo.dataType));
            return;
    }
}

void copySingleLayerMaskData(dai::SegmentationMask& outputMask,
                             dai::NNData& nnData,
                             dai::TensorInfo& tensorInfo,
                             dai::SegmentationParserConfig& config,
                             std::shared_ptr<spdlog::async_logger>& logger) {
    logger->trace("Parsing single layer segmentation mask.");
    const size_t stepSize = static_cast<size_t>(config.stepSize);
    const size_t maskWidth = static_cast<size_t>(tensorInfo.getWidth());
    const size_t maskHeight = static_cast<size_t>(tensorInfo.getHeight());
    const size_t strideH = tensorInfo.getHeightStride();
    const size_t strideW = tensorInfo.getWidthStride();
    const int dstWidth = static_cast<int>(maskWidth / stepSize);
    const int dstHeight = static_cast<int>(maskHeight / stepSize);
    span<uint8_t> dst = outputMask.prepareMask(dstWidth, dstHeight);

    auto nnMemory = nnData.data;
    const auto srcBytes = nnMemory->getData();
    const auto tensorSpan = srcBytes.subspan(tensorInfo.offset, tensorInfo.getTensorSize());
    const uint8_t* tensorBase = tensorSpan.data();

    int outY = 0;
    for(size_t h = 0; h < maskHeight; h += stepSize, ++outY) {
        const size_t rowBase = strideH * h;
        int outX = 0;
        for(size_t w = 0; w < maskWidth; w += stepSize, ++outX) {
            const size_t idx = rowBase + (strideW * w);
            int32_t v;
            std::memcpy(&v, tensorBase + idx, sizeof(v));
            v = std::max(v, 0);
            v = std::min(v, 255);
            dst[(outY * dstWidth) + outX] = static_cast<uint8_t>(v);
        }
    }
}

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai
