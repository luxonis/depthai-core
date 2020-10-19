#pragma once

#include <limits>
#include <unordered_map>
#include <vector>

#include "Buffer.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"

namespace dai {

class NNData : public Buffer {
    static constexpr int DATA_ALIGNMENT = 64;
    std::shared_ptr<RawBuffer> serialize() const;
    RawNNData& rawNn;

   public:
    NNData();
    NNData(std::shared_ptr<RawNNData> ptr);
    ~NNData() = default;

    // store the data
    // uint8_t
    std::unordered_map<std::string, std::vector<std::uint8_t>> u8Data;
    // FP16
    std::unordered_map<std::string, std::vector<std::uint16_t>> fp16Data;

    // Expose
    // uint8_t
    void setLayer(const std::string& name, std::vector<std::uint8_t> data);
    void setLayer(const std::string& name, const std::vector<int>& data);

    // fp16
    void setLayer(const std::string& name, std::vector<float> data);
    void setLayer(const std::string& name, std::vector<double> data);

    // getters
    bool getLayer(const std::string& name, TensorInfo& tensor);
    bool hasLayer(const std::string& name);
    bool getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype);
    // uint8
    std::vector<std::uint8_t> getLayerUInt8(const std::string& name);
    // fp16
    std::vector<float> getLayerFp16(const std::string& name);

    // first layer
    std::vector<std::uint8_t> getFirstLayerUInt8();
    std::vector<float> getFirstLayerFp16();
};

}  // namespace dai
