#pragma once
#include <spdlog/async_logger.h>

#include <cstddef>
#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "fp16/fp16.h"
namespace dai {
class NNDataViewer {
   public:
    std::shared_ptr<dai::Memory> data;
    dai::TensorInfo tensor;
    std::shared_ptr<spdlog::async_logger> logger;

    // Factors to multiply with before the vectors
    struct FactorsBefore {
        int32_t h;
        int32_t w;
        int32_t c;
    };

    FactorsBefore factorsBefore;

    NNDataViewer(dai::TensorInfo tensor, std::shared_ptr<dai::Memory> data, std::shared_ptr<spdlog::async_logger> logger)
        : data{data}, tensor{tensor}, logger{logger} {};
    bool build() {
        if(tensor.strides.size() < 2) {
            logger->error("Tensor doesn't have enough strides. Number of strides: {}, expected: {}", tensor.strides.size(), 2);
            return false;
        }
        if(tensor.strides[0] == 0 || tensor.strides[1] == 0) {
            logger->error("Tensor strides should not be set to zero. Strides are {} {}", tensor.strides[0], tensor.strides[1]);
            return false;
        }
        switch(tensor.order) {
            case TensorInfo::StorageOrder::NCHW:
                if(tensor.dims[0] != 1) {
                    logger->error("NCHW is only supported in Detection Parser if N is 1. It is {}", tensor.dims[0]);
                    return false;
                }
                if(tensor.strides.size() != 4) {
                    logger->error("Invalid number of strides: {}, expected: {}", tensor.strides.size(), 4);
                    return false;
                }
                factorsBefore.c = tensor.strides[1];
                factorsBefore.h = tensor.strides[2];
                factorsBefore.w = tensor.getDataTypeSize();
                break;
            case TensorInfo::StorageOrder::NHWC:
                if(tensor.dims[0] != 1) {
                    logger->error("NHWC is only supported in Detection Parser if N is 1. It is {}", tensor.dims[0]);
                    return false;
                }
                if(tensor.strides.size() != 4) {
                    logger->error("Invalid number of strides: {}, expected: {}", tensor.strides.size(), 4);
                    return false;
                }
                factorsBefore.h = tensor.strides[1];
                factorsBefore.w = tensor.strides[2];
                factorsBefore.c = tensor.getDataTypeSize();
                break;
            case TensorInfo::StorageOrder::HCW:
                factorsBefore.h = tensor.strides[0];
                factorsBefore.c = tensor.strides[1];
                factorsBefore.w = tensor.getDataTypeSize();
                break;

            case TensorInfo::StorageOrder::HWC:
                factorsBefore.h = tensor.strides[0];
                factorsBefore.w = tensor.strides[1];
                factorsBefore.c = tensor.getDataTypeSize();
                break;
            case TensorInfo::StorageOrder::CHW:
                factorsBefore.c = tensor.strides[0];
                factorsBefore.h = tensor.strides[1];
                factorsBefore.w = tensor.getDataTypeSize();
                break;

            case TensorInfo::StorageOrder::CWH:
                factorsBefore.c = tensor.strides[0];
                factorsBefore.w = tensor.strides[1];
                factorsBefore.h = tensor.getDataTypeSize();
                break;

            case TensorInfo::StorageOrder::WCH:
                factorsBefore.w = tensor.strides[0];
                factorsBefore.c = tensor.strides[1];
                factorsBefore.h = tensor.getDataTypeSize();
                break;

            case TensorInfo::StorageOrder::WHC:
                factorsBefore.w = tensor.strides[0];
                factorsBefore.h = tensor.strides[1];
                factorsBefore.c = tensor.getDataTypeSize();
                break;
            case TensorInfo::StorageOrder::NHCW:
            case TensorInfo::StorageOrder::NC:
            case TensorInfo::StorageOrder::CN:
            case TensorInfo::StorageOrder::H:
            case TensorInfo::StorageOrder::W:
            case TensorInfo::StorageOrder::C:
            default:
                logger->error("Storage order not supported in NNDataViewer");
                return false;
        }
        return sanity_check();
    }

    bool sanity_check() {
        if(data->getSize() < (tensor.offset + (tensor.dims[0] * tensor.strides[0]))) {
            logger->error(
                "Underlying data does not hold enough data for the tensor to be contained.\
                Tensor size: {}, Tensor offset: {}, Data type size: {}, Data size: {} ",
                tensor.dims[0] * tensor.strides[0],
                tensor.offset,
                tensor.getDataTypeSize(),
                data->getSize());
            return false;
        }
        if(tensor.dims.size() < 2) {
            logger->error("Number of dimensions for the input tensor is expected to be at least 2. It is {}", tensor.dims.size());
            return false;
        }
        return true;
    };

    bool copyToChannelMajor(std::vector<float>& dst) {
        int width = tensor.getWidth();
        int height = tensor.getHeight();
        int channels = tensor.getChannels();

        if(width <= 0 || height <= 0 || channels <= 0) {
            logger->error("Invalid tensor dimensions for channel-major copy. Channels: {}, height: {}, width: {}.", channels, height, width);
            return false;
        }

        const size_t planeSize = static_cast<size_t>(width) * static_cast<size_t>(height);
        const size_t totalSize = planeSize * static_cast<size_t>(channels);
        dst.assign(totalSize, float{0});

        const uint8_t* tensorBase = data->getData().data() + tensor.offset;

        for(int c = 0; c < channels; ++c) {
            float* destRow = dst.data() + static_cast<size_t>(c) * planeSize;
            const uint8_t* channelPtr = tensorBase + factorsBefore.c * c;
            for(int h = 0; h < height; ++h) {
                const uint8_t* rowPtr = channelPtr + factorsBefore.h * h;
                const size_t rowOffset = static_cast<size_t>(h) * static_cast<size_t>(width);
                for(int w = 0; w < width; ++w) {
                    const uint8_t* elementPtr = rowPtr + factorsBefore.w * w;
                    float value = readRawValue(elementPtr);
                    float dequantized = (value - tensor.qpZp) * tensor.qpScale;
                    destRow[rowOffset + static_cast<size_t>(w)] = static_cast<float>(dequantized);
                }
            }
        }

        return true;
    }

    inline float get(int c, int h, int w) {
        // If this turns out to be slow, use a function pointer instead and point to the right getter at build time
        int32_t index = tensor.offset + factorsBefore.h * h + factorsBefore.w * w + factorsBefore.c * c;
#ifdef DEPTHAI_SAFE_NN_DATA_ACCESS
        logger->trace("Offset {}, fbH {}, fbW {}, fbC {}, h {}, w {}, c{}", tensor.offset, factorsBefore.h, factorsBefore.w, factorsBefore.c, h, w, c);
        if(index > data->getSize()) {
            logger->error("Out of bound access. Size is {}, index is {}", data->getSize(), index);
            return 0.0;
        }
#endif

        switch(tensor.dataType) {
            case TensorInfo::DataType::U8F: {
                uint8_t dataOut = data->getData()[index];
                return (static_cast<float>(dataOut) - tensor.qpZp) * tensor.qpScale;
            }
            case TensorInfo::DataType::I8: {
                int8_t dataOut = static_cast<int8_t>(data->getData()[index]);
                return (static_cast<float>(dataOut) - tensor.qpZp) * tensor.qpScale;
            }
            case TensorInfo::DataType::INT: {
                int32_t dataOut = reinterpret_cast<int32_t*>(data->getData().data())[index / sizeof(int32_t)];
                return (static_cast<float>(dataOut) - tensor.qpZp) * tensor.qpScale;
            }
            case TensorInfo::DataType::FP16: {
                int16_t dataOut = reinterpret_cast<int16_t*>(data->getData().data())[index / sizeof(int16_t)];
                return (fp16_ieee_to_fp32_value(dataOut) - tensor.qpZp) * tensor.qpScale;
            }
            case TensorInfo::DataType::FP32: {
                float dataOut = reinterpret_cast<float*>(data->getData().data())[index / sizeof(float)];
                return (static_cast<float>(dataOut) - tensor.qpZp) * tensor.qpScale;
            }
            case TensorInfo::DataType::FP64:
            default: {
                return 0.0f;
            }
        }
    }

   private:
    inline float readRawValue(const uint8_t* dataPtr) const {
        switch(tensor.dataType) {
            case TensorInfo::DataType::U8F:
                return static_cast<float>(*dataPtr);
            case TensorInfo::DataType::I8:
                return static_cast<float>(*reinterpret_cast<const int8_t*>(dataPtr));
            case TensorInfo::DataType::INT:
                return static_cast<float>(*reinterpret_cast<const int32_t*>(dataPtr));
            case TensorInfo::DataType::FP16:
                return fp16_ieee_to_fp32_value(*reinterpret_cast<const uint16_t*>(dataPtr));
            case TensorInfo::DataType::FP32:
                return *reinterpret_cast<const float*>(dataPtr);
            case TensorInfo::DataType::FP64:
            default:
                return 0.0f;
        }
    }
};
}  // namespace dai
