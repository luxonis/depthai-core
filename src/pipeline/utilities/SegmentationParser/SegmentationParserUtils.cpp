#include "SegmentationParserUtils.hpp"

#include <fp16/fp16.h>
#include <spdlog/async_logger.h>

#include <cstring>
#include <memory>
#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

template <typename T>
inline T loadT(const uint8_t* p) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "Segmentation type must be trivially copyable");
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

template <typename T>
void parseNHWCMask(std::vector<uint8_t>& dst, dai::NNData& nnData, dai::TensorInfo& tensorInfo, dai::SegmentationParserConfig& config) {
    const uint8_t* tensorBase = nnData.data->getData().data() + tensorInfo.offset;
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();
    const float qpZp = tensorInfo.qpZp;
    const float qpScale = tensorInfo.qpScale;

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const T quantizedThreshold = static_cast<T>(config.confidenceThreshold / qpScale + qpZp);
    const int step = config.stepSize;

    int dstWidth = width / step;
    int dstHeight = height / step;
    dst.resize(dstWidth * dstHeight);
    uint8_t* dstPtr = dst.data();

    const auto addr = reinterpret_cast<std::uintptr_t>(tensorBase);
    const bool aligned = (addr % alignof(T)) == 0;
    const bool stridesOk = (strideC % (int)sizeof(T) == 0) && (strideH % (int)sizeof(T) == 0) && (strideW % (int)sizeof(T) == 0);
    const bool typed = aligned && stridesOk;

    if(typed) {  // typed pointer access
        printf("typed parseNHWC\n");

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
                dstPtr[rowOffset + outX] = bestClass;
            }
        }
    } else {
        int outY = 0;
        for(int h = 0; h < height; h += step, ++outY) {
            const uint8_t* rowBase = tensorBase + strideH * h;

            int outX = 0;
            int rowOffset = outY * dstWidth;
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
                dstPtr[rowOffset + outX] = bestClass;
            }
        }
    }
}

template <typename T>
void parseNCHWMask(std::vector<uint8_t>& dst, dai::NNData& nnData, dai::TensorInfo& tensorInfo, dai::SegmentationParserConfig& config) {
    const uint8_t* tensorBase = nnData.data->getData().data() + tensorInfo.offset;
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();
    const float qpZp = tensorInfo.qpZp;
    const float qpScale = tensorInfo.qpScale;

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const T quantizedThreshold = static_cast<T>(config.confidenceThreshold / qpScale + qpZp);
    const int step = config.stepSize;

    int dstWidth = width / step;
    int dstHeight = height / step;
    dst.resize(dstWidth * dstHeight, 255);
    uint8_t* dstPtr = dst.data();

    const auto addr = reinterpret_cast<std::uintptr_t>(tensorBase);
    const bool aligned = (addr % alignof(T)) == 0;
    const bool stridesOk = (strideC % (int)sizeof(T) == 0) && (strideH % (int)sizeof(T) == 0) && (strideW % (int)sizeof(T) == 0);
    const bool typed = aligned && stridesOk;

    std::vector<T> bestVals(dstWidth * dstHeight, quantizedThreshold);

    if(typed) {  // typed pointer access
        printf("typed parseNCHW\n");
        const T* baseT = reinterpret_cast<const T*>(tensorBase);
        const int strideC_T = strideC / (int)sizeof(T);
        const int strideH_T = strideH / (int)sizeof(T);
        const int strideW_T = strideW / (int)sizeof(T);

        for(int c = 0; c < channels; c++) {
            const T* channelBase = baseT + strideC_T * c;

            int outY = 0;
            for(int h = 0; h < height; h += step, ++outY) {
                const T* rowBase = channelBase + strideH_T * h;
                int outX = 0;
                int rowOffset = outY * dstWidth;

                for(int w = 0; w < width; w += step, ++outX) {
                    T v = *(rowBase + strideW_T * w);
                    int coordinateIndex = rowOffset + outX;
                    if(v > bestVals[coordinateIndex]) {
                        bestVals[coordinateIndex] = v;
                        dstPtr[coordinateIndex] = c;
                    }
                }
            }
        }

    } else {
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

                    if(v > bestVals[coordinateIndex]) {
                        bestVals[coordinateIndex] = v;
                        dstPtr[coordinateIndex] = c;
                    }
                }
            }
        }
    }
}

template <typename T>
void thresholdAndArgmaxTensor(std::vector<uint8_t>& dst,
                              dai::NNData& nnData,
                              dai::TensorInfo& tensorInfo,
                              dai::SegmentationParserConfig& config,
                              std::shared_ptr<spdlog::async_logger>& logger) {
    if(tensorInfo.order == dai::TensorInfo::StorageOrder::NCHW || tensorInfo.order == dai::TensorInfo::StorageOrder::CHW) {
        parseNCHWMask<T>(dst, nnData, tensorInfo, config);
    } else if(tensorInfo.order == dai::TensorInfo::StorageOrder::NHWC || tensorInfo.order == dai::TensorInfo::StorageOrder::HWC) {
        parseNHWCMask<T>(dst, nnData, tensorInfo, config);
    } else {
        logger->error("Unsupported storage order for segmentation parsing: {}", static_cast<int>(tensorInfo.order));
        return;
    }
}

void thresholdAndArgmaxFp16Tensor(std::vector<uint8_t>& dst,
                                  dai::NNData& nnData,
                                  dai::TensorInfo& tensorInfo,
                                  dai::SegmentationParserConfig& config) {  // RVC2 path
    const uint8_t* tensorBase = nnData.data->getData().data() + tensorInfo.offset;
    const int channels = tensorInfo.getChannels();
    const int width = tensorInfo.getWidth();
    const int height = tensorInfo.getHeight();
    const float qpZp = tensorInfo.qpZp;
    const float qpScale = tensorInfo.qpScale;

    const int strideC = tensorInfo.getChannelStride();
    const int strideH = tensorInfo.getHeightStride();
    const int strideW = tensorInfo.getWidthStride();

    const float quantizedThreshold = float(config.confidenceThreshold / qpScale + qpZp);
    const int step = config.stepSize;

    int dstWidth = width / step;
    int dstHeight = height / step;
    dst.resize(dstWidth * dstHeight, 255);
    uint8_t* dstPtr = dst.data();

    std::vector<float> bestVals(dstWidth * dstHeight, quantizedThreshold);

    printf("rvc2 path, assumes NCHW\n");
    for(int c = 0; c < channels; c++) {
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
                    dstPtr[coordinateIndex] = c;
                }
            }
        }
    }
}

void parseSegmentationMask(dai::NNData& nnData,
                           dai::TensorInfo& tensorInfo,
                           std::vector<uint8_t>& outputMask,
                           dai::SegmentationParserConfig& config,
                           std::shared_ptr<spdlog::async_logger>& logger) {
    using namespace std::chrono;

    switch(tensorInfo.dataType) {
        case dai::TensorInfo::DataType::I8:
            thresholdAndArgmaxTensor<int8_t>(outputMask, nnData, tensorInfo, config, logger);
            break;
        case dai::TensorInfo::DataType::U8F:
            thresholdAndArgmaxTensor<uint8_t>(outputMask, nnData, tensorInfo, config, logger);
            break;
        case dai::TensorInfo::DataType::INT:
            thresholdAndArgmaxTensor<int32_t>(outputMask, nnData, tensorInfo, config, logger);
            break;
        case dai::TensorInfo::DataType::FP32:
            thresholdAndArgmaxTensor<float>(outputMask, nnData, tensorInfo, config, logger);
            break;
        case dai::TensorInfo::DataType::FP16:
            thresholdAndArgmaxFp16Tensor(outputMask, nnData, tensorInfo, config);
            break;
        case dai::TensorInfo::DataType::FP64:
        default:
            logger->error("Unsupported data type for segmentation parsing: {}", static_cast<int>(tensorInfo.dataType));
            return;
    }
}

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai
