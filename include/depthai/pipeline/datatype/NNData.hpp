#pragma once

#include <algorithm>
#include <chrono>
#include <limits>
#include <unordered_map>
#include <vector>

#include "Buffer.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"

// Optional - XTensor support
#ifdef DEPTHAI_HAVE_XTENSOR_SUPPORT
    #if defined(__clang__)
        #if __has_warning("-Wswitch-enum")
            #pragma clang diagnostic push
            #pragma clang diagnostic ignored "-Wswitch-enum"
        #endif
    #elif defined(__GNUC__)
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wswitch-enum"
    #elif defined(_MSC_VER)
        #pragma warning(push)
        #pragma warning(disable : 4061)
    #endif
    #if defined(ON)
        #define _ON_DEF ON
        #undef ON
    #endif
    #include "xtensor/xadapt.hpp"
    #include "xtensor/xarray.hpp"
    #if defined(_ON_DEF)
        #define ON _ON_DEF
    #endif
    #if defined(__clang__)
        #if __has_warning("-Wswitch-enum")
            #pragma clang diagnostic pop
        #endif
    #elif defined(__GNUC__)
        #pragma GCC diagnostic pop
    #elif defined(_MSC_VER)
        #pragma warning(pop)
    #endif
#endif

namespace dai {

/**
 * NNData message. Carries tensors and their metadata
 */
class NNData : public Buffer {
    static constexpr int DATA_ALIGNMENT = 64;
    [[deprecated("Use 'addTensor()' instead")]] std::shared_ptr<RawBuffer> serialize() const override;
    static uint16_t fp32_to_fp16(float);
    static float fp16_to_fp32(uint16_t);
    RawNNData& rawNn;

    // store the data
    // uint8_t
    std::unordered_map<std::string, std::vector<std::uint8_t>> u8Data;
    // FP16
    std::unordered_map<std::string, std::vector<std::uint16_t>> fp16Data;

   public:
    /**
     * Construct NNData message.
     */
    NNData();
    explicit NNData(std::shared_ptr<RawNNData> ptr);
    virtual ~NNData() = default;

    // Expose
    // uint8_t
    /**
     * Set a layer with datatype U8.
     * @param name Name of the layer
     * @param data Data to store
     */
    [[deprecated("Use 'addTensor()' instead")]] NNData& setLayer(const std::string& name, std::vector<std::uint8_t> data);

    /**
     * Set a layer with datatype U8. Integers are cast to bytes.
     * @param name Name of the layer
     * @param data Data to store
     */
    [[deprecated("Use 'addTensor()' instead")]] NNData& setLayer(const std::string& name, const std::vector<int>& data);

    // fp16
    /**
     * Set a layer with datatype FP16. Float values are converted to FP16.
     * @param name Name of the layer
     * @param data Data to store
     */
    [[deprecated("Use 'addTensor()' instead")]] NNData& setLayer(const std::string& name, std::vector<float> data);

    /**
     * Set a layer with datatype FP16. Double values are converted to FP16.
     * @param name Name of the layer
     * @param data Data to store
     */
    [[deprecated("Use 'addTensor()' instead")]] NNData& setLayer(const std::string& name, std::vector<double> data);

    // getters
    /**
     * @returns Names of all layers added
     */
    std::vector<std::string> getAllLayerNames() const;

    /**
     * @returns All layers and their information
     */
    std::vector<TensorInfo> getAllLayers() const;

    /**
     * Retrieve layers tensor information
     * @param name Name of the layer
     * @param[out] tensor Outputs tensor information of that layer
     * @returns True if layer exists, false otherwise
     */
    [[deprecated("Use 'getTensor()' instead")]] bool getLayer(const std::string& name, TensorInfo& tensor) const;

    /**
     * Checks if given layer exists
     * @param name Name of the layer
     * @returns True if layer exists, false otherwise
     */
    bool hasLayer(const std::string& name) const;

    /**
     * Retrieve datatype of a layers tensor
     * @param name Name of the layer
     * @param[out] datatype Datatype of layers tensor
     * @returns True if layer exists, false otherwise
     */
    bool getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype) const;

    // uint8
    /**
     * Convenience function to retrieve U8 data from layer
     * @param name Name of the layer
     * @returns U8 binary data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<std::uint8_t> getLayerUInt8(const std::string& name) const;

    // fp16
    /**
     * Convenience function to retrieve float values from layers FP16 tensor
     * @param name Name of the layer
     * @returns Float data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<float> getLayerFp16(const std::string& name) const;

    // int32
    /**
     * Convenience function to retrieve INT32 values from layers tensor
     * @param name Name of the layer
     * @returns INT32 data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<std::int32_t> getLayerInt32(const std::string& name) const;

    // first layer
    /**
     * Convenience function to retrieve U8 data from first layer
     * @returns U8 binary data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<std::uint8_t> getFirstLayerUInt8() const;

    /**
     * Convenience function to retrieve float values from first layers FP16 tensor
     * @returns Float data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<float> getFirstLayerFp16() const;

    /**
     * Convenience function to retrieve INT32 values from first layers tensor
     * @returns INT32 data
     */
    [[deprecated("Use 'getTensor()' instead")]] std::vector<std::int32_t> getFirstLayerInt32() const;

    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves image timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used mostly for debugging
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /**
     * Retrieves image sequence number
     */
    int64_t getSequenceNum() const;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    NNData& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    NNData& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    NNData& setSequenceNum(int64_t sequenceNum);

    /**
     * Get the datatype of a given tensor
     * @returns TensorInfo::DataType tensor datatype
     */
    TensorInfo::DataType getTensorDatatype(const std::string& name);

// Optional - XTensor support
#ifdef DEPTHAI_HAVE_XTENSOR_SUPPORT
    /**
     * @note This API is only available if XTensor support is enabled
     *
     * Set a layer with datatype FP16. Double values are converted to FP16.
     * @param name Name of the layer
     * @param data Data to store
     */
    template <typename T = double>
    NNData& addTensor(const std::string& name, const std::vector<T>& data) {
        return addTensor<T>(name, xt::adapt(data, std::vector<size_t>{1, data.size()}));
    };

    /**
     * @note This API is only available if XTensor support is enabled
     *
     * Add a tensor. Float values are converted to FP16 and integers are cast to bytes.
     * @param name Name of the tensor
     * @param tensor Tensor to store
     */
    template <typename T = double>
    NNData& addTensor(const std::string& name, const xt::xarray<T>& tensor) {
        static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "Tensor type needs to be integral or floating point");

        // Get size in bytes of the converted tensor data, u8 for integral and fp16 for floating point
        const size_t sConvertedData = std::is_integral<T>::value ? tensor.size() : 2 * tensor.size();

        // Append bytes so that each new tensor is DATA_ALIGNMENT aligned
        size_t remainder = (rawNn.data.end() - rawNn.data.begin()) % DATA_ALIGNMENT;
        if(remainder > 0) {
            rawNn.data.insert(rawNn.data.end(), DATA_ALIGNMENT - remainder, 0);
        }

        // Then get offset to beginning of data
        size_t offset = rawNn.data.end() - rawNn.data.begin();

        // Reserve space
        rawNn.data.resize(offset + sConvertedData);

        // Convert data to u8 or fp16 and write to rawNn.data
        if(std::is_integral<T>::value) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                rawNn.data.data()[i + offset] = (uint8_t)tensor.data()[i];
            }
        } else {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(uint16_t*)(&rawNn.data.data()[2 * i + offset]) = fp32_to_fp16(tensor.data()[i]);
            }
        }

        // Add entry in tensors
        TensorInfo info;
        info.name = name;
        info.offset = static_cast<unsigned int>(offset);
        info.dataType = std::is_integral<T>::value ? TensorInfo::DataType::U8F : TensorInfo::DataType::FP16;
        info.numDimensions = tensor.dimension();
        for(uint32_t i = 0; i < tensor.dimension(); i++) {
            info.dims.push_back(tensor.shape()[i]);
            info.strides.push_back(tensor.strides()[i]);
        }

        rawNn.tensors.push_back(info);
        return *this;
    }

    /**
     * @note This API is only available if XTensor support is enabled
     *
     * Convenience function to retrieve values from a tensor
     * @returns xt::xarray<T> tensor
     */
    template <typename T>
    xt::xarray<T> getTensor(const std::string& name) {
        const auto it = std::find_if(rawNn.tensors.begin(), rawNn.tensors.end(), [&name](const TensorInfo& ti) { return ti.name == name; });

        if(it == rawNn.tensors.end()) throw std::runtime_error("Tensor does not exist");

        std::vector<size_t> dims;
        for(const auto v : it->dims) {
            dims.push_back(v);
        }

        std::vector<size_t> strides;
        for(const auto v : it->strides) {
            strides.push_back(v);
        }

        xt::xarray<T, xt::layout_type::row_major> tensor(dims);
        if(it->dataType == TensorInfo::DataType::U8F) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                tensor.data()[i] = rawNn.data.data()[it->offset + i];
            }
        } else {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                tensor.data()[i] = fp16_to_fp32(*(uint16_t*)&rawNn.data.data()[it->offset + 2 * i]);
            }
        }

        return tensor;
    }

    /**
     * @note This API is only available if XTensor support is enabled
     *
     * Convenience function to retrieve values from the first tensor
     * @returns xt::xarray<T> tensor
     */
    template <typename T>
    xt::xarray<T> getFirstTensor() {
        if(!rawNn.tensors.empty()) {
            return getTensor<T>(rawNn.tensors[0].name);
        }

        return {};
    }
#else
    /**
     * Add a tensor. Float values are converted to FP16 and integers are cast to bytes.
     * @param name Name of the tensor
     * @param tensor Tensor to store
     */
    template <typename T = double>
    NNData& addTensor(const std::string& name, const std::vector<T>& tensor) {
        static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "Tensor type needs to be integral or floating point");

        // Get size in bytes of the converted tensor data, u8 for integral and fp16 for floating point
        const size_t sConvertedData = std::is_integral<T>::value ? tensor.size() : 2 * tensor.size();

        // Append bytes so that each new tensor is DATA_ALIGNMENT aligned
        size_t remainder = (rawNn.data.end() - rawNn.data.begin()) % DATA_ALIGNMENT;
        if(remainder > 0) {
            rawNn.data.insert(rawNn.data.end(), DATA_ALIGNMENT - remainder, 0);
        }

        // Then get offset to beginning of data
        size_t offset = rawNn.data.end() - rawNn.data.begin();

        // Reserve space
        rawNn.data.resize(offset + sConvertedData);

        // Convert data to u8 or fp16 and write to rawNn.data
        if(std::is_integral<T>::value) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                rawNn.data.data()[i + offset] = (uint8_t)tensor.data()[i];
            }
        } else {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(uint16_t*)(&rawNn.data.data()[2 * i + offset]) = fp32_to_fp16(tensor.data()[i]);
            }
        }

        // Add entry in tensors
        TensorInfo info;
        info.dataType = std::is_integral<T>::value ? TensorInfo::DataType::U8F : TensorInfo::DataType::FP16;
        info.numDimensions = 1;
        info.dims.push_back(tensor.size());
        info.strides.push_back(std::is_integral<T>::value ? sizeof(uint8_t) : sizeof(uint16_t));
        info.name = name;
        info.offset = static_cast<unsigned int>(offset);
        rawNn.tensors.push_back(info);
        return *this;
    }

    /**
     * Convenience function to retrieve values from a tensor
     * @returns std::vector<T> tensor
     */
    template <typename T>
    std::vector<T> getTensor(const std::string& name) {
        const auto it = std::find_if(rawNn.tensors.begin(), rawNn.tensors.end(), [&name](const TensorInfo& ti) { return ti.name == name; });

        if(it == rawNn.tensors.end()) throw std::runtime_error("Tensor does not exist");

        size_t dims = it->dims[0];

        std::vector<T> tensor(dims);
        if(it->dataType == TensorInfo::DataType::U8F) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                tensor.data()[i] = rawNn.data.data()[it->offset + i];
            }
        } else {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                tensor.data()[i] = fp16_to_fp32(*(uint16_t*)&rawNn.data.data()[it->offset + 2 * i]);
            }
        }

        return tensor;
    }

    /**
     * Convenience function to retrieve values from the first tensor
     * @returns std::vector<T> tensor
     */
    template <typename T>
    std::vector<T> getFirstTensor() {
        if(!rawNn.tensors.empty()) {
            return getTensor<T>(rawNn.tensors[0].name);
        }

        return {};
    }
#endif
};

}  // namespace dai
