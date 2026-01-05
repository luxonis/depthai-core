
#pragma once

// std
#include <cstdint>
#include <ostream>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// TensorInfo structure
struct TensorInfo {
    enum class StorageOrder : int {
        NHWC = 0x4213,
        NHCW = 0x4231,
        NCHW = 0x4321,
        HWC = 0x213,
        CHW = 0x321,
        WHC = 0x123,
        HCW = 0x231,
        WCH = 0x132,
        CWH = 0x312,
        NC = 0x43,
        CN = 0x34,
        C = 0x3,
        H = 0x2,
        W = 0x1,
    };

    enum class DataType : int {
        FP16 = 0,  // Half precision floating point
        U8F = 1,   // Unsigned byte
        INT = 2,   // Signed integer (4 byte)
        FP32 = 3,  // Single precision floating point
        I8 = 4,    // Signed byte
        FP64 = 5,  // Double precision floating point
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
            case DataType::FP64:
                return sizeof(double);
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

    std::size_t getTensorSize() {
        uint32_t i = 0;

        // Handle the edge case if all dimensions are 1
        // In this case, the size is the size of the data type
        bool non1dim = false;
        for(i = 0; i < dims.size(); i++) {
            if(dims[i] != 1) {
                non1dim = true;
                break;
            }
        }
        if(!non1dim) {
            return getDataTypeSize();
        }

        // Use the first non zero stride
        for(i = 0; i < strides.size(); i++) {
            if(strides[i] > 0) {
                break;
            }
        }
        return dims[i] * strides[i];
    }

    int getChannels() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return dims[3];
            case StorageOrder::NHCW:
                return dims[2];
            case StorageOrder::NCHW:
                return dims[1];
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

    int getChannelStride() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return strides[3];
            case StorageOrder::NHCW:
                return strides[2];
            case StorageOrder::NCHW:
                return strides[1];
            case StorageOrder::HWC:
                return strides[2];
            case StorageOrder::CHW:
                return strides[0];
            case StorageOrder::WHC:
                return strides[0];
            case StorageOrder::HCW:
                return strides[1];
            case StorageOrder::WCH:
                return strides[1];
            case StorageOrder::CWH:
                return strides[0];
            case StorageOrder::C:
                return strides[0];
            case StorageOrder::NC:
                return strides[1];
            case StorageOrder::CN:
                return strides[0];
            case StorageOrder::H:
            case StorageOrder::W:
            default:
                return 0;
        }
    }

    int getWidthStride() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return strides[2];
            case StorageOrder::NHCW:
                return strides[3];
            case StorageOrder::NCHW:
                return strides[3];
            case StorageOrder::HWC:
                return strides[1];
            case StorageOrder::CHW:
                return strides[2];
            case StorageOrder::WHC:
                return strides[0];
            case StorageOrder::HCW:
                return strides[1];
            case StorageOrder::WCH:
                return strides[0];
            case StorageOrder::CWH:
                return strides[1];
            case StorageOrder::W:
                return strides[0];
            case StorageOrder::NC:
            case StorageOrder::CN:
            case StorageOrder::H:
            case StorageOrder::C:
            default:
                return 0;
        }
    }

    int getHeightStride() {
        validateStorageOrder();
        switch(order) {
            case StorageOrder::NHWC:
                return strides[1];
            case StorageOrder::NHCW:
                return strides[1];
            case StorageOrder::NCHW:
                return strides[2];
            case StorageOrder::HWC:
                return strides[0];
            case StorageOrder::CHW:
                return strides[1];
            case StorageOrder::WHC:
                return strides[1];
            case StorageOrder::HCW:
                return strides[0];
            case StorageOrder::WCH:
                return strides[1];
            case StorageOrder::CWH:
                return strides[1];
            case StorageOrder::H:
                return strides[0];
            case StorageOrder::C:
            case StorageOrder::NC:
            case StorageOrder::CN:
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

inline std::ostream& operator<<(std::ostream& os, const TensorInfo::StorageOrder& so) {
    switch(so) {
        case TensorInfo::StorageOrder::NHWC:
            os << "NHWC";
            break;
        case TensorInfo::StorageOrder::NHCW:
            os << "NHCW";
            break;
        case TensorInfo::StorageOrder::NCHW:
            os << "NCHW";
            break;
        case TensorInfo::StorageOrder::HWC:
            os << "HWC";
            break;
        case TensorInfo::StorageOrder::CHW:
            os << "CHW";
            break;
        case TensorInfo::StorageOrder::WHC:
            os << "WHC";
            break;
        case TensorInfo::StorageOrder::HCW:
            os << "HCW";
            break;
        case TensorInfo::StorageOrder::WCH:
            os << "WCH";
            break;
        case TensorInfo::StorageOrder::CWH:
            os << "CWH";
            break;
        case TensorInfo::StorageOrder::NC:
            os << "NC";
            break;
        case TensorInfo::StorageOrder::CN:
            os << "CN";
            break;
        case TensorInfo::StorageOrder::C:
            os << "C";
            break;
        case TensorInfo::StorageOrder::H:
            os << "H";
            break;
        case TensorInfo::StorageOrder::W:
            os << "W";
            break;
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const TensorInfo::DataType& dt) {
    switch(dt) {
        case TensorInfo::DataType::FP16:
            os << "FP16";
            break;
        case TensorInfo::DataType::U8F:
            os << "U8F";
            break;
        case TensorInfo::DataType::INT:
            os << "INT";
            break;
        case TensorInfo::DataType::FP32:
            os << "FP32";
            break;
        case TensorInfo::DataType::I8:
            os << "I8";
            break;
        case TensorInfo::DataType::FP64:
            os << "FP64";
            break;
    }
    return os;
}

}  // namespace dai
