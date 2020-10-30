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
    explicit NNData(std::shared_ptr<RawNNData> ptr);
    virtual ~NNData() = default;

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
    std::vector<std::string> getAllLayerNames() const;
    std::vector<TensorInfo> getAllLayers() const;
    bool getLayer(const std::string& name, TensorInfo& tensor) const;
    bool hasLayer(const std::string& name) const;
    bool getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype) const;
    // uint8
    std::vector<std::uint8_t> getLayerUInt8(const std::string& name) const;
    // fp16
    std::vector<float> getLayerFp16(const std::string& name) const;

    // first layer
    std::vector<std::uint8_t> getFirstLayerUInt8() const;
    std::vector<float> getFirstLayerFp16() const;
};

}  // namespace dai
