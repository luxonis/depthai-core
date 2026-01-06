#include "SegmentationParserUtils.hpp"

#include <fp16/fp16.h>
#include <spdlog/async_logger.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

constexpr uint8_t BACKGROUND_INDEX = 255;
constexpr int MAX_CLASS_INDEX = 254;

template <typename T>
inline T loadT(const uint8_t* p) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "Segmentation type must be trivially copyable");
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

template <typename T>
void parseNHWCMask(span<uint8_t> dst, const uint8_t* tensorBase, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config, const bool typed) {
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const T quantizedThreshold = static_cast<T>(config.confidenceThreshold / tensorInfo.qpScale + tensorInfo.qpZp);

    const int step = config.stepSize;
    const int dstWidth = width / step;
    if(typed) {
        const T* baseT = reinterpret_cast<const T*>(tensorBase);
        const int strideC_T = strideC / (int)sizeof(T);
        const int strideH_T = strideH / (int)sizeof(T);
        const int strideW_T = strideW / (int)sizeof(T);

        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const T* rowBase = baseT + strideH_T * h;

            int outX = 0;
            int rowOffset = outY * dstWidth;
            for(int w = 0; w < width; w += step, ++outX) {
                const T* pixelBase = rowBase + strideW_T * w;

                uint8_t bestClass = 255;
                T bestVal = quantizedThreshold;

                for(uint8_t c = 0; c < channels; ++c) {
                    T v = *(pixelBase + strideC_T * c);
                    if(v > bestVal) {
                        bestVal = v;
                        bestClass = c;
                    }
                }
                dst[rowOffset + outX] = bestClass;
            }
        }
    } else {  // memory safe per element loading (slower)
        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const uint8_t* rowBase = tensorBase + strideH * h;

            int rowOffset = outY * dstWidth;
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const uint8_t* pixelBase = rowBase + strideW * w;

                uint8_t bestClass = 255;
                T bestVal = quantizedThreshold;

                for(uint8_t c = 0; c < channels; ++c) {
                    const uint8_t* p = pixelBase + strideC * c;
                    T v = loadT<T>(p);
                    if(v > bestVal) {
                        bestVal = v;
                        bestClass = c;
                    }
                }
                dst[rowOffset + outX] = bestClass;
            }
        }
    }
}

template <typename T>
void parseNCHWMask(span<uint8_t> dst, const uint8_t* tensorBase, dai::TensorInfo& tensorInfo, const dai::SegmentationParserConfig& config, const bool typed) {
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const T quantizedThreshold = static_cast<T>(config.confidenceThreshold / tensorInfo.qpScale + tensorInfo.qpZp);
    static thread_local std::vector<T> bestValues;
    bestValues.resize(dst.size());
    std::fill(bestValues.begin(), bestValues.end(), quantizedThreshold);

    const int step = config.stepSize;
    const int dstWidth = width / step;
    if(typed) {
        const T* baseT = reinterpret_cast<const T*>(tensorBase);
        const int strideC_T = strideC / (int)sizeof(T);
        const int strideH_T = strideH / (int)sizeof(T);
        const int strideW_T = strideW / (int)sizeof(T);

        for(int c = 0; c < channels; c++) {
            const T* channelBase = baseT + strideC_T * c;

            int outY = 0;
            for(int h = 0; h < height; h += step, ++outY) {
                const T* rowBase = channelBase + strideH_T * h;
                int rowOffset = outY * dstWidth;

                int outX = 0;
                for(int w = 0; w < width; w += step, ++outX) {
                    T v = *(rowBase + strideW_T * w);
                    int coordinateIndex = rowOffset + outX;
                    if(v > bestValues[coordinateIndex]) {
                        bestValues[coordinateIndex] = v;
                        dst[coordinateIndex] = c;
                    }
                }
            }
        }

    } else {  // memory safe per element loading (slower)
        for(int c = 0; c < channels; c++) {
            const uint8_t* channelBase = tensorBase + strideC * c;

            int outY = 0;
            for(int h = 0; h < height; h += step, ++outY) {
                const uint8_t* rowBase = channelBase + strideH * h;
                int outX = 0;
                for(int w = 0; w < width; w += step, ++outX) {
                    const uint8_t* p = rowBase + strideW * w;
                    T v = loadT<T>(p);
                    int coordinateIndex = outY * dstWidth + outX;

                    if(v > bestValues[coordinateIndex]) {
                        bestValues[coordinateIndex] = v;
                        dst[coordinateIndex] = c;
                    }
                }
            }
        }
    }
}

template <typename T>
void thresholdAndArgmaxTensor(dai::SegmentationMask& dstMask,
                              dai::NNData& nnData,
                              dai::TensorInfo& tensorInfo,
                              dai::SegmentationParserConfig& config,
                              std::shared_ptr<spdlog::async_logger>& logger) {
    const uint8_t* tensorBase = nnData.data->getData().data() + tensorInfo.offset;
    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const auto addr = reinterpret_cast<std::uintptr_t>(tensorBase);
    const bool aligned = (addr % alignof(T)) == 0;
    const bool stridesOk = (strideC % (int)sizeof(T) == 0) && (strideH % (int)sizeof(T) == 0) && (strideW % (int)sizeof(T) == 0);
    const bool useTypedPointer = aligned && stridesOk;

    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();
    const int dstWidth = width / config.stepSize;
    const int dstHeight = height / config.stepSize;
    auto dstData = dstMask.prepareMask(static_cast<size_t>(dstWidth), static_cast<size_t>(dstHeight));
    std::fill(dstData.begin(), dstData.end(), BACKGROUND_INDEX);

    if(tensorInfo.order == dai::TensorInfo::StorageOrder::NCHW || tensorInfo.order == dai::TensorInfo::StorageOrder::CHW) {
        logger->trace("Parsing NCHW/CHW segmentation mask");
        parseNCHWMask<T>(dstData, tensorBase, tensorInfo, config, useTypedPointer);
    } else if(tensorInfo.order == dai::TensorInfo::StorageOrder::NHWC || tensorInfo.order == dai::TensorInfo::StorageOrder::HWC) {
        logger->trace("Parsing NHWC/HWC segmentation mask");
        parseNHWCMask<T>(dstData, tensorBase, tensorInfo, config, useTypedPointer);
    } else {
        logger->error("Unsupported storage order for segmentation parsing: {}", static_cast<int>(tensorInfo.order));
        return;
    }
}

void thresholdAndArgmaxFp16Tensor(dai::SegmentationMask& dst, dai::NNData& nnData, dai::TensorInfo& tensorInfo, dai::SegmentationParserConfig& config) {
    const uint8_t* tensorBase = nnData.data->getData().data() + tensorInfo.offset;
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();

    const float quantizedThreshold = float(config.confidenceThreshold / tensorInfo.qpScale + tensorInfo.qpZp);

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const int step = config.stepSize;
    const int dstWidth = width / step;
    const int dstHeight = height / step;

    static thread_local std::vector<float> bestVals;
    bestVals.resize(static_cast<size_t>(dstWidth * dstHeight));
    std::fill(bestVals.begin(), bestVals.end(), quantizedThreshold);

    auto dstData = dst.prepareMask(static_cast<size_t>(dstWidth), static_cast<size_t>(dstHeight));
    std::fill(dstData.begin(), dstData.end(), BACKGROUND_INDEX);

    for(int c = 0; c < channels; c++) {  // Optimized for NCHW since FP16 is asociated with RVC2
        const uint8_t* channelBase = tensorBase + strideC * c;

        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const uint8_t* rowBase = channelBase + strideH * h;
            int outX = 0;
            for(int w = 0; w < width; w += step, ++outX) {
                const uint8_t* p = rowBase + strideW * w;
                float v = fp16_ieee_to_fp32_value(loadT<uint16_t>(p));
                int coordinateIndex = outY * dstWidth + outX;

                if(v > bestVals[coordinateIndex]) {
                    bestVals[coordinateIndex] = v;
                    dstData[coordinateIndex] = c;
                }
            }
        }
    }
}

template void thresholdAndArgmaxTensor<int8_t>(
    dai::SegmentationMask&, dai::NNData&, dai::TensorInfo&, dai::SegmentationParserConfig&, std::shared_ptr<spdlog::async_logger>&);
template void thresholdAndArgmaxTensor<uint8_t>(
    dai::SegmentationMask&, dai::NNData&, dai::TensorInfo&, dai::SegmentationParserConfig&, std::shared_ptr<spdlog::async_logger>&);
template void thresholdAndArgmaxTensor<int32_t>(
    dai::SegmentationMask&, dai::NNData&, dai::TensorInfo&, dai::SegmentationParserConfig&, std::shared_ptr<spdlog::async_logger>&);
template void thresholdAndArgmaxTensor<float>(
    dai::SegmentationMask&, dai::NNData&, dai::TensorInfo&, dai::SegmentationParserConfig&, std::shared_ptr<spdlog::async_logger>&);

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai
