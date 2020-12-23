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
    rawNn.tensors = {};

    // offset

    // Add to data and align to 64 bytes and create an entry in tensor describing its content
    for(const auto& kv : u8Data) {
        // Get offset first
        size_t offset = rawNn.data.end() - rawNn.data.begin();

        const auto* data = kv.second.data();
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

        // Add entry in tensors
        TensorInfo info;
        info.dataType = TensorInfo::DataType::U8F;
        info.numDimensions = 0;
        info.name = kv.first;
        info.offset = offset;
        rawNn.tensors.push_back(info);
    }
    for(const auto& kv : fp16Data) {
        // Get offset first
        size_t offset = rawNn.data.end() - rawNn.data.begin();

        const auto* data = reinterpret_cast<const std::uint8_t*>(kv.second.data());
        rawNn.data.insert(rawNn.data.end(), data, data + kv.second.size() * sizeof(std::uint16_t));

        // calculate how many bytes to add
        size_t remainder = kv.second.size() % DATA_ALIGNMENT;
        size_t toAdd = 0;
        if(remainder > 0) {
            toAdd = DATA_ALIGNMENT - remainder;
        }

        // add padding bytes
        rawNn.data.insert(rawNn.data.end(), toAdd, 0);

        // Add entry in tensors
        TensorInfo info;
        info.dataType = TensorInfo::DataType::U8F;
        info.numDimensions = 0;
        info.name = kv.first;
        info.offset = offset;
        rawNn.tensors.push_back(info);
    }

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
std::vector<std::string> NNData::getAllLayerNames() const {
    std::vector<std::string> names;
    for(const auto& t : rawNn.tensors) {
        names.push_back(t.name);
    }
    return names;
}

std::vector<TensorInfo> NNData::getAllLayers() const {
    return rawNn.tensors;
}

bool NNData::getLayer(const std::string& name, TensorInfo& tensor) const {
    for(const auto& t : rawNn.tensors) {
        if(t.name == name) {
            tensor = t;
            return true;
        }
    }
    return false;
}

bool NNData::hasLayer(const std::string& name) const {
    for(const auto& tensor : rawNn.tensors) {
        if(tensor.name == name) return true;
    }
    return false;
}

bool NNData::getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype) const {
    TensorInfo tensor;
    if(getLayer(name, tensor)) {
        datatype = tensor.dataType;
        return true;
    }
    return false;
}

// uint8
std::vector<std::uint8_t> NNData::getLayerUInt8(const std::string& name) const {
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
std::vector<float> NNData::getLayerFp16(const std::string& name) const {
    // find layer name and its offset
    TensorInfo tensor;
    if(getLayer(name, tensor)) {
        if(tensor.dataType == TensorInfo::DataType::FP16) {
            // Total data size = last dimension * last stride
            if(tensor.numDimensions > 0) {
                std::size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];
                std::size_t numElements = size / 2;  // FP16

                std::vector<float> data;
                auto* pFp16Data = reinterpret_cast<std::uint16_t*>(&rawNn.data[tensor.offset]);
                for(std::size_t i = 0; i < numElements; i++) {
                    data.push_back(fp16_ieee_to_fp32_value(pFp16Data[i]));
                }
                return data;
            }
        }
    }
    return {};
}

// uint8
std::vector<std::uint8_t> NNData::getFirstLayerUInt8() const {
    // find layer name and its offset
    if(!rawNn.tensors.empty()) {
        return getLayerUInt8(rawNn.tensors[0].name);
    }

    return {};
}

// fp16
std::vector<float> NNData::getFirstLayerFp16() const {
    // find layer name and its offset
    if(!rawNn.tensors.empty()) {
        return getLayerFp16(rawNn.tensors[0].name);
    }
    return {};
}

}  // namespace dai
