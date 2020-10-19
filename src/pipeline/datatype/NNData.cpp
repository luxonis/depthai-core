#include "depthai/pipeline/datatype/NNData.hpp"

#include <limits>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "fp16/fp16.h"

namespace dai {

NNData::NNData() : Buffer(std::make_shared<RawNNData>()), rawNn(*dynamic_cast<RawNNData*>(raw.get())) {}
NNData::NNData(std::shared_ptr<RawNNData> ptr) : Buffer(ptr), rawNn(*ptr.get()) {}

std::shared_ptr<RawBuffer> NNData::serialize() const {
    // get data from u8Data and fp16Data and place properly into the underlying raw buffer

    // Get smallest size to put last
    size_t smallestSize = std::numeric_limits<size_t>::max();
    const std::uint8_t* pSmallest = nullptr;
    for(const auto& kv : u8Data) {
        if(kv.second.size() < smallestSize) {
            smallestSize = kv.second.size() * sizeof(std::uint8_t);
            pSmallest = kv.second.data();
        }
    }
    for(const auto& kv : fp16Data) {
        if(kv.second.size() < smallestSize) {
            smallestSize = kv.second.size() * sizeof(std::uint16_t);
            pSmallest = reinterpret_cast<const std::uint8_t*>(kv.second.data());
        }
    }

    // Add to data, but skip smallest (and align to 64 bytes)
    for(const auto& kv : u8Data) {
        if(kv.second.data() != pSmallest) {
            auto* data = kv.second.data();
            rawNn.data.insert(rawNn.data.end(), data, data + kv.second.size() * sizeof(std::uint8_t));
            // insert alignment bytes (64B alignment)

            // calculate how many bytes to add
            size_t remainder = kv.second.size() % DATA_ALIGNMENT;
            size_t toAdd = 0;
            if(remainder > 0) {
                toAdd = DATA_ALIGNMENT - remainder;
            }

            // add padding bytes
            rawNn.data.insert(rawNn.data.end(), toAdd, 0);
        }
    }
    for(const auto& kv : fp16Data) {
        if(reinterpret_cast<const uint8_t*>(kv.second.data()) != pSmallest) {
            auto* data = reinterpret_cast<const std::uint8_t*>(kv.second.data());
            rawNn.data.insert(rawNn.data.end(), data, data + kv.second.size() * sizeof(std::uint16_t));

            // calculate how many bytes to add
            size_t remainder = kv.second.size() % DATA_ALIGNMENT;
            size_t toAdd = 0;
            if(remainder > 0) {
                toAdd = DATA_ALIGNMENT - remainder;
            }

            // add padding bytes
            rawNn.data.insert(rawNn.data.end(), toAdd, 0);
        }
    }

    // Add last remaining data without padding
    rawNn.data.insert(rawNn.data.end(), pSmallest, pSmallest + smallestSize);

    return raw;
}

// setters
// uint8_t
void NNData::setLayer(const std::string& name, std::vector<std::uint8_t> data) {
    u8Data[name] = std::move(data);
}
void NNData::setLayer(const std::string& name, const std::vector<int>& data) {
    u8Data[name] = std::vector<std::uint8_t>(data.size());
    for(unsigned i = 0; i < data.size(); i++) {
        u8Data[name][i] = static_cast<std::uint8_t>(data[i]);
    }
}

// fp16
void NNData::setLayer(const std::string& name, std::vector<float> data) {
    fp16Data[name] = std::vector<std::uint16_t>(data.size());
    for(unsigned i = 0; i < data.size(); i++) {
        fp16Data[name][i] = fp16_ieee_from_fp32_value(data[i]);
    }
}
void NNData::setLayer(const std::string& name, std::vector<double> data) {
    fp16Data[name] = std::vector<std::uint16_t>(data.size());
    for(unsigned i = 0; i < data.size(); i++) {
        fp16Data[name][i] = fp16_ieee_from_fp32_value(data[i]);
    }
}

// getters
bool NNData::getLayer(const std::string& name, TensorInfo& tensor) {
    for(const auto& t : rawNn.tensors) {
        if(t.name == name) {
            tensor = t;
            return true;
        }
    }
    return false;
}

bool NNData::hasLayer(const std::string& name) {
    for(const auto& tensor : rawNn.tensors) {
        if(tensor.name == name) return true;
    }
    return false;
}

bool NNData::getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype) {
    TensorInfo tensor;
    if(getLayer(name, tensor)) {
        datatype = tensor.dataType;
        return true;
    }
    return false;
}

// uint8
std::vector<std::uint8_t> NNData::getLayerUInt8(const std::string& name) {
    // std::vector<std::uint8_t> data;
    // find layer name and its offset
    TensorInfo tensor;
    if(getLayer(name, tensor)) {
        if(tensor.dataType == TensorInfo::DataType::U8F) {
            // Total data size = last dimension * last stride
            if(tensor.numDimensions > 0) {
                size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];
                auto beg = rawNn.data.begin() + tensor.offset;
                auto end = beg + size;
                return {beg, end};
            }
        }
    }
    return {};
}

// fp16
std::vector<float> NNData::getLayerFp16(const std::string& name) {
    // find layer name and its offset
    TensorInfo tensor;
    if(getLayer(name, tensor)) {
        if(tensor.dataType == TensorInfo::DataType::FP16) {
            // Total data size = last dimension * last stride
            if(tensor.numDimensions > 0) {
                size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];

                std::vector<float> data;
                auto* pFp16Data = reinterpret_cast<std::uint16_t*>(&rawNn.data[tensor.offset]);
                for(unsigned int i = 0; i < size; i++) {
                    data.push_back(fp16_ieee_to_fp32_value(pFp16Data[i]));
                }
                return data;
            }
        }
    }
    return {};
}

// uint8
std::vector<std::uint8_t> NNData::getFirstLayerUInt8() {
    // find layer name and its offset
    if(rawNn.tensors.size() > 0) {
        TensorInfo tensor = rawNn.tensors[0];
        if(tensor.dataType == TensorInfo::DataType::FP16) {
            // Total data size = last dimension * last stride
            if(tensor.dataType == TensorInfo::DataType::U8F) {
                // Total data size = last dimension * last stride
                if(tensor.numDimensions > 0) {
                    size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];
                    auto beg = rawNn.data.begin() + tensor.offset;
                    auto end = beg + size;
                    return {beg, end};
                }
            }
        }
    }

    return {};
}

// fp16
std::vector<float> NNData::getFirstLayerFp16() {
    // find layer name and its offset
    if(rawNn.tensors.size() > 0) {
        TensorInfo tensor = rawNn.tensors[0];
        if(tensor.dataType == TensorInfo::DataType::FP16) {
            // Total data size = last dimension * last stride
            if(tensor.numDimensions > 0) {
                size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];
                size /= sizeof(std::uint16_t);  // divide by size of fp16

                std::vector<float> data;
                auto* pFp16Data = reinterpret_cast<std::uint16_t*>(&rawNn.data[tensor.offset]);
                for(unsigned int i = 0; i < size; i++) {
                    data.push_back(fp16_ieee_to_fp32_value(pFp16Data[i]));
                }
                return data;
            }
        }
    }
    return {};
}

}  // namespace dai
