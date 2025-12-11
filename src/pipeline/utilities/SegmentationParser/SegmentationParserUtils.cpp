#include "SegmentationParserUtils.hpp"

#include <spdlog/async_logger.h>

#include <memory>
#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "pipeline/utilities/NNDataViewer.hpp"

namespace dai {
namespace utilities {
namespace SegmentationParserUtils {

template <typename T>
void thresholdAndArgmaxTensor(std::vector<uint8_t>& dst, dai::NNDataViewer& viewer, int stepSize, float threshold) {
    dai::TensorInfo& tensor = viewer.tensor;
    const uint8_t* tensorBase = viewer.data->getData().data() + tensor.offset;

    const int channels = tensor.getChannels();
    const int width = tensor.getWidth();
    const int height = tensor.getHeight();
    const float qpZp = tensor.qpZp;
    const float qpScale = tensor.qpScale;

    const T quantizedThreshold = static_cast<T>(threshold / qpScale + qpZp);

    const int strideC = viewer.factorsBefore.c;
    const int strideH = viewer.factorsBefore.h;
    const int strideW = viewer.factorsBefore.w;

    // optimization for NCHW:
    // if (contiguousRow) {
    // const T* srcRow = reinterpret_cast<const T*>(rowBase);
    // for(int w = 0; w < width; w += stepSize) {
    //     dstRow[w] = (static_cast<float>(srcRow[w]) - qpZp) * qpScale;
    // }
    // bool contiguousRow = (strideW == sizeof(T));

    int dstWidth = width / stepSize;
    int dstHeight = height / stepSize;
    dst.resize(dstWidth * dstHeight);
    uint8_t* dstPtr = dst.data();

    for(int h = 0; h < height; h += stepSize) {
        const uint8_t* rowBase = tensorBase + strideH * h;

        for(int w = 0; w < width; w += stepSize) {
            const uint8_t* widthBase = rowBase + strideW * w;

            uint8_t bestClass = 255;
            T bestVal = quantizedThreshold;
            for(int c = 0; c < channels; ++c) {
                const T* valPtr = reinterpret_cast<const T*>(widthBase + strideC * c);
                if(*valPtr > bestVal) {
                    bestVal = *valPtr;
                    bestClass = static_cast<uint8_t>(c);
                }
            }
            dstPtr[(h * dstWidth + w) / stepSize] = bestClass;
        }
    }
}

void parseSegmentationMask(const dai::NNData& nnData,
                           dai::TensorInfo& tensorInfo,
                           std::vector<uint8_t>& outputMask,
                           dai::SegmentationParserConfig& config,
                           std::shared_ptr<spdlog::async_logger>& logger) {
    using namespace std::chrono;

    NNDataViewer viewer(tensorInfo, nnData.data, logger);  // used to get stride factors
    if(!viewer.build()) {
        logger->error("Failed to build NNDataViewer for tensor {}. Skipping processing.", tensorInfo.name);
        return;
    }

    // logger->warn("NNDataViewer build took {}ms", duration_cast<microseconds>(steady_clock::now() - tNNviewerStart).count() / 1000);  // 0ms

    switch(tensorInfo.dataType) {
        case dai::TensorInfo::DataType::I8:
            thresholdAndArgmaxTensor<int8_t>(outputMask, viewer, config.stepSize, config.confidenceThreshold);
            break;
        case dai::TensorInfo::DataType::U8F:
            thresholdAndArgmaxTensor<uint8_t>(outputMask, viewer, config.stepSize, config.confidenceThreshold);
            break;
        case dai::TensorInfo::DataType::INT:
            thresholdAndArgmaxTensor<int32_t>(outputMask, viewer, config.stepSize, config.confidenceThreshold);
            break;
            //     thresholdAndArgmaxTensor<uint16_t>(outputMask, viewer, config.stepSize, config.confidenceThreshold);
            //     for(size_t i = 0; i < outputMask.size(); i++) {
            //         outputMask[i] = fp16_ieee_to_fp32_value(outputMask[i]);
            //     }
            //     break;
        case dai::TensorInfo::DataType::FP32:
            thresholdAndArgmaxTensor<float>(outputMask, viewer, config.stepSize, config.confidenceThreshold);
            break;
        case dai::TensorInfo::DataType::FP16:
        case dai::TensorInfo::DataType::FP64:
        default:
            logger->error("Unsupported data type for segmentation parsing: {}", static_cast<int>(tensorInfo.dataType));
            return;
    }
}

}  // namespace SegmentationParserUtils
}  // namespace utilities
}  // namespace dai