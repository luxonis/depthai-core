#include "depthai/pipeline/datatype/NNData.hpp"

#include <cassert>
#include <limits>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/VectorMemory.hpp"

#ifndef RVC2_FW
    #include "fp16/fp16.h"
#endif

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/NNData.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

NNData::~NNData() = default;

void NNData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::NNData;
}

NNData::NNData(size_t size) : NNData() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}

//         // Add entry in tensors
//         // TODO(themarpe) - refactor with proper way off specifying tensors
//         TensorInfo info;
//         info.dataType = dataType;
//         info.numDimensions = 1;
//         info.dims.push_back(kv.second.size());
//         info.strides.push_back(sizeofTensorInfoDataType(dataType));
//         info.name = kv.first;
//         info.offset = static_cast<unsigned int>(offset);
//         tensors.push_back(info);
//     }

//         // Add entry in tensors
//         // TODO(themarpe) - refactor with proper way off specifying tensors
//         TensorInfo info;
//         info.dataType = dataType;
//         info.numDimensions = 1;
//         info.dims.push_back(kv.second.size());
//         info.strides.push_back(sizeofTensorInfoDataType(dataType));
//         info.name = kv.first;
//         info.offset = static_cast<unsigned int>(offset);
//         tensors.push_back(info);
//     }

#ifndef RVC2_FW
uint16_t NNData::fp32_to_fp16(float value) {
    return fp16_ieee_from_fp32_value(value);
};

float NNData::fp16_to_fp32(uint16_t value) {
    return fp16_ieee_to_fp32_value(value);
};
#endif

span<std::uint8_t> NNData::emplaceTensor(TensorInfo& tensor) {
    // TODO - look into returning an xtensor adaptor pre RVC3 merge
    size_t offset = data->getSize();
    auto tensorSize = tensor.getTensorSize();
    size_t reminder = tensorSize % DATA_ALIGNMENT;
    auto tensorSizeAligned = tensorSize;
    if(reminder != 0) {
        tensorSizeAligned += DATA_ALIGNMENT - reminder;
    }
    tensor.offset = offset;
    tensors.push_back(tensor);
    // TODO - this might not be safe/viable with all types of memory
    data->setSize(offset + tensorSizeAligned);
    return data->getData().subspan(offset, tensorSize);
}

std::vector<std::string> NNData::getAllLayerNames() const {
    std::vector<std::string> names;
    for(const auto& t : tensors) {
        names.push_back(t.name);
    }
    return names;
}

std::vector<TensorInfo> NNData::getAllLayers() const {
    return tensors;
}

std::optional<TensorInfo> NNData::getTensorInfo(const std::string& name) const {
    for(const auto& t : tensors) {
        if(t.name == name) {
            return t;
        }
    }
    return std::nullopt;
}

bool NNData::getLayer(const std::string& name, TensorInfo& tensor) const {
    for(const auto& t : tensors) {
        if(t.name == name) {
            tensor = t;
            return true;
        }
    }
    return false;
}

bool NNData::hasLayer(const std::string& name) const {
    for(const auto& tensor : tensors) {
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

TensorInfo::DataType NNData::getTensorDatatype(const std::string& name) {
    const auto it = std::find_if(tensors.begin(), tensors.end(), [&name](const TensorInfo& ti) { return ti.name == name; });

    if(it == tensors.end()) throw std::runtime_error("Tensor does not exist");

    return it->dataType;
};

TensorInfo::DataType NNData::getFirstTensorDatatype() {
    if(tensors.empty()) {
        throw std::runtime_error("Tensor does not exist");
    }

    return tensors.front().dataType;
};

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair NNData::serializeSchema() const {
    return utility::getProtoSchema<NNData>();
}

std::vector<std::uint8_t> NNData::serializeProto(bool metadataOnly) const {
    return utility::serializeProto(utility::getProtoMessage(this, metadataOnly));
}

void NNData::deserializeProto(const std::vector<std::uint8_t>& bytes) {
    utility::deserializeProtoMessage(*this, bytes);
}
#endif

#ifdef DEPTHAI_XTENSOR_SUPPORT
NNData& NNData::addTensor(const std::string& name, const std::vector<int>& tensor) {  // Out of line: inline, it instantiated xtensor templates in every unit
    return addTensor<int>(name, tensor, dai::TensorInfo::DataType::INT);
}

NNData& NNData::addTensor(const std::string& name, const std::vector<uint16_t>& tensor) {
    return addTensor<uint16_t>(name, tensor, dai::TensorInfo::DataType::FP16);
}

NNData& NNData::addTensor(const std::string& name, const std::vector<float>& tensor) {
    return addTensor<float>(name, tensor, dai::TensorInfo::DataType::FP32);
}

NNData& NNData::addTensor(const std::string& name, const std::vector<double>& tensor) {
    return addTensor<double>(name, tensor, dai::TensorInfo::DataType::FP64);
}

NNData& NNData::addTensor(const std::string& name, const std::vector<std::int8_t>& tensor) {
    return addTensor<std::int8_t>(name, tensor, dai::TensorInfo::DataType::I8);
}

NNData& NNData::addTensor(const std::string& name, const std::vector<std::uint8_t>& tensor) {
    return addTensor<std::uint8_t>(name, tensor, dai::TensorInfo::DataType::U8F);
}
#endif

}  // namespace dai
