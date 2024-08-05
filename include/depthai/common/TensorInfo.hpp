
#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// TensorInfo structure
struct TensorInfo {
    enum class StorageOrder : int {
        NHWC = 4213,
        NHCW = 4231,
        NCHW = 4321,
        HWC = 213,
        CHW = 321,
        WHC = 123,
        HCW = 231,
        WCH = 132,
        CWH = 312,
        NC = 43,
        CN = 34,
        C = 3,
        H = 2,
        W = 1,
    };

    enum class DataType : int {
        FP16 = 0,  // Half precision floating point
        U8F = 1,   // Unsigned byte
        INT = 2,   // Signed integer (4 byte)
        FP32 = 3,  // Single precision floating point
        I8 = 4,    // Signed byte
    };

    void validateStorageOrder() {
        switch(order) {
            case StorageOrder::NHWC:
            case StorageOrder::NHCW:
            case StorageOrder::NCHW:
                if(dims.size() < 4) {
                    throw std::runtime_error("Not enough dimensions (needs at least 4) for the storage order in TensorInfo");
                }
                break;
            case StorageOrder::HWC:
            case StorageOrder::CHW:
            case StorageOrder::WHC:
            case StorageOrder::HCW:
            case StorageOrder::WCH:
            case StorageOrder::CWH:
                if(dims.size() < 3) {
                    throw std::runtime_error("Not enough dimensions (needs at least 3) for the storage order in TensorInfo");
                }
                break;
            case StorageOrder::NC:
            case StorageOrder::CN:
                if(dims.size() < 2) {
                    throw std::runtime_error("Not enough dimensions (needs at least 2) for the storage order in TensorInfo");
                }
                break;
            case StorageOrder::C:
            case StorageOrder::H:
            case StorageOrder::W:
                if(dims.size() < 1) {
                    throw std::runtime_error("Not enough dimensions (needs at least 1) for the storage order in TensorInfo");
                }
                break;
            default:
                throw std::runtime_error("Invalid storage order type in TensorInfo");
        }
    }

    int getDataTypeSize() {
        switch(dataType) {
            case DataType::U8F:
            case DataType::I8:
                return sizeof(uint8_t);
            case DataType::FP16:
                return sizeof(uint16_t);
            case DataType::INT:
            case DataType::FP32:
                return sizeof(float);
            default:
                return 0;
                break;
        }
    }

    int getWidth() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return dims[2];
            case StorageOrder::NHCW:
                return dims[3];
            case StorageOrder::NCHW:
                return dims[3];
            case StorageOrder::HWC:
                return dims[1];
            case StorageOrder::CHW:
                return dims[2];
            case StorageOrder::WHC:
                return dims[1];
            case StorageOrder::HCW:
                return dims[2];
            case StorageOrder::WCH:
                return dims[0];
            case StorageOrder::CWH:
                return dims[1];
            case StorageOrder::W:
                return dims[0];
            case StorageOrder::NC:
            case StorageOrder::CN:
            case StorageOrder::C:
            case StorageOrder::H:
            default:
                return 0;
        }
    }

    int getHeight() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return dims[1];
            case StorageOrder::NHCW:
                return dims[1];
            case StorageOrder::NCHW:
                return dims[2];
            case StorageOrder::HWC:
                return dims[0];
            case StorageOrder::CHW:
                return dims[1];
            case StorageOrder::WHC:
                return dims[1];
            case StorageOrder::HCW:
                return dims[0];
            case StorageOrder::WCH:
                return dims[2];
            case StorageOrder::CWH:
                return dims[2];
            case StorageOrder::H:
                return dims[0];
            case StorageOrder::NC:
            case StorageOrder::CN:
            case StorageOrder::C:
            case StorageOrder::W:
            default:
                return 0;
        }
    }

    int getChannels() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return dims[3];
            case StorageOrder::NHCW:
                return dims[2];
            case StorageOrder::NCHW:
                return dims[3];
            case StorageOrder::HWC:
                return dims[2];
            case StorageOrder::CHW:
                return dims[0];
            case StorageOrder::WHC:
                return dims[2];
            case StorageOrder::HCW:
                return dims[1];
            case StorageOrder::WCH:
                return dims[1];
            case StorageOrder::CWH:
                return dims[0];
            case StorageOrder::C:
                return dims[0];
            case StorageOrder::NC:
                return dims[1];
            case StorageOrder::CN:
                return dims[0];
            case StorageOrder::H:
            case StorageOrder::W:
            default:
                return 0;
        }
    }

    StorageOrder order = StorageOrder::NCHW;
    DataType dataType = DataType::FP16;
    unsigned int numDimensions = 0;
    std::vector<unsigned> dims;
    std::vector<unsigned> strides;
    std::string name;
    unsigned int offset = 0;

    // quantization params (deqValue = (val - qpZp) * scale )
    bool quantization = false;
    float qpScale = 1;
    float qpZp = 0;
};

DEPTHAI_SERIALIZE_EXT(TensorInfo, order, dataType, numDimensions, dims, strides, name, offset, quantization, qpScale, qpZp);

}  // namespace dai
