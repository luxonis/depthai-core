#pragma once

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iterator>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include "Buffer.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/utility/VectorMemory.hpp"
#include "depthai/utility/span.hpp"

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
#ifdef DEPTHAI_XTENSOR_SUPPORT
    #include "xtensor/containers/xadapt.hpp"
    #include "xtensor/containers/xarray.hpp"
    #include "xtensor/misc/xmanipulation.hpp"
#endif
#if defined(_ON_DEF)
    #define ON _ON_DEF
#endif
#if defined(__clang__)
    #if __has_warning("-Wswitch-enum")
        #ifdef __clang__
            #pragma clang diagnostic pop
        #endif
    #endif
#elif defined(__GNUC__)
    #pragma GCC diagnostic pop
#elif defined(_MSC_VER)
    #pragma warning(pop)
#endif

namespace dai {

/**
 * NNData message. Carries tensors and their metadata
 */
class NNData : public Buffer {
    static constexpr int DATA_ALIGNMENT = 64;
    static uint16_t fp32_to_fp16(float);
    static float fp16_to_fp32(uint16_t);

   public:
    std::vector<TensorInfo> tensors;
    unsigned int batchSize;
    std::optional<ImgTransformation> transformation;
    /**
     * Construct NNData message.
     */
    NNData() = default;
    /**
     * Construct NNData with a preallocated data size.
     */
    NNData(size_t size);
    virtual ~NNData();

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
     * Retrieve tensor information
     * @param name Name of the tensor
     * @returns Tensor information
     */
    std::optional<TensorInfo> getTensorInfo(const std::string& name) const;

    // TODO(Morato) - deprecate this
    /**
     * Retrieve layers tensor information
     * @param name Name of the layer
     * @param[out] tensor Outputs tensor information of that layer
     * @returns True if layer exists, false otherwise
     */
    bool getLayer(const std::string& name, TensorInfo& tensor) const;

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

    /**
     * Get the datatype of a given tensor
     * @returns TensorInfo::DataType tensor datatype
     */
    TensorInfo::DataType getTensorDatatype(const std::string& name);

    /**
     * Get the datatype of the first tensor
     * @returns TensorInfo::DataType tensor datatype
     */
    TensorInfo::DataType getFirstTensorDatatype();

    /**
     * Emplace a tensor
     * This function allocates memory for the tensor and return over the said memory.
     * It is up to the caller to fill the memory out with meaningful data.
     * @return Span over the allocated memory
     */
    span<std::uint8_t> emplaceTensor(TensorInfo& tensor);

#ifdef DEPTHAI_XTENSOR_SUPPORT
    /**
     * @brief Add a tensor to this NNData object.
     * The provided array is stored as a 1xN tensor where N is the length of the array.
     *
     * @param name: Name of the tensor
     * @param data: array
     * @return NNData&: reference to this object
     */
    template <typename _Ty = double>
    /**
     * Add a tensor from a 1xN vector with explicit data type.
     */
    NNData& addTensor(const std::string& name, const std::vector<_Ty>& data, dai::TensorInfo::DataType dataType) {
        return addTensor<_Ty>(name, xt::adapt(data, std::vector<size_t>{1, data.size()}), dataType);
    };
    // addTensor vector dispatch
    template <typename _Ty = double>
    /**
     * Add a tensor from a vector with type-based data type selection.
     */
    NNData& addTensor(const std::string& name, const std::vector<_Ty>& tensor) {
        if constexpr(std::is_same<_Ty, int>::value) {
            return addTensor<int>(name, tensor, dai::TensorInfo::DataType::INT);
        } else if constexpr(std::is_same<_Ty, uint16_t>::value) {
            return addTensor<uint16_t>(name, tensor, dai::TensorInfo::DataType::FP16);
        } else if constexpr(std::is_same<_Ty, float>::value) {
            return addTensor<float>(name, tensor, dai::TensorInfo::DataType::FP32);
        } else if constexpr(std::is_same<_Ty, double>::value) {
            return addTensor<double>(name, tensor, dai::TensorInfo::DataType::FP64);
        } else if constexpr(std::is_same<_Ty, std::int8_t>::value) {
            return addTensor<std::int8_t>(name, tensor, dai::TensorInfo::DataType::I8);
        } else if constexpr(std::is_same<_Ty, std::uint8_t>::value) {
            return addTensor<std::uint8_t>(name, tensor, dai::TensorInfo::DataType::U8F);
        } else {
            throw std::runtime_error("Unsupported datatype");
        }
    }

    /**
     * Add an INT tensor from int data.
     */
    NNData& addTensor(const std::string& name, const std::vector<int>& tensor) {
        return addTensor<int>(name, tensor, dai::TensorInfo::DataType::INT);
    };
    /**
     * Add a FP16 tensor from uint16_t data.
     */
    NNData& addTensor(const std::string& name, const std::vector<uint16_t>& tensor) {
        return addTensor<uint16_t>(name, tensor, dai::TensorInfo::DataType::FP16);
    };
    /**
     * Add a FP32 tensor from float data.
     */
    NNData& addTensor(const std::string& name, const std::vector<float>& tensor) {
        return addTensor<float>(name, tensor, dai::TensorInfo::DataType::FP32);
    };
    /**
     * Add a FP64 tensor from double data.
     */
    NNData& addTensor(const std::string& name, const std::vector<double>& tensor) {
        return addTensor<double>(name, tensor, dai::TensorInfo::DataType::FP64);
    };
    /**
     * Add an INT8 tensor from int8 data.
     */
    NNData& addTensor(const std::string& name, const std::vector<std::int8_t>& tensor) {
        return addTensor<std::int8_t>(name, tensor, dai::TensorInfo::DataType::I8);
    };
    /**
     * Add a U8F tensor from uint8 data.
     */
    NNData& addTensor(const std::string& name, const std::vector<std::uint8_t>& tensor) {
        return addTensor<std::uint8_t>(name, tensor, dai::TensorInfo::DataType::U8F);
    };

    // addTensor dispatch
    template <typename _Ty = double>
    /**
     * Add a tensor from an xt::xarray with type-based data type selection.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<_Ty>& tensor) {
        if constexpr(std::is_same<_Ty, int>::value) {
            return addTensor<int>(name, tensor, dai::TensorInfo::DataType::INT);
        } else if(std::is_same<_Ty, uint16_t>::value) {
            return addTensor<uint16_t>(name, tensor, dai::TensorInfo::DataType::FP16);
        } else if constexpr(std::is_same<_Ty, float>::value) {
            return addTensor<float>(name, tensor, dai::TensorInfo::DataType::FP32);
        } else if constexpr(std::is_same<_Ty, double>::value) {
            return addTensor<double>(name, tensor, dai::TensorInfo::DataType::FP64);
        } else if constexpr(std::is_same<_Ty, std::int8_t>::value) {
            return addTensor<std::int8_t>(name, tensor, dai::TensorInfo::DataType::I8);
        } else if constexpr(std::is_same<_Ty, std::uint8_t>::value) {
            return addTensor<std::uint8_t>(name, tensor, dai::TensorInfo::DataType::U8F);
        } else {
            throw std::runtime_error("Unsupported datatype");
        }
    }

    /**
     * Add an INT tensor from int xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<int>& tensor) {
        return addTensor<int>(name, tensor, dai::TensorInfo::DataType::INT);
    };
    /**
     * Add a FP16 tensor from uint16_t xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<uint16_t>& tensor) {
        return addTensor<uint16_t>(name, tensor, dai::TensorInfo::DataType::FP16);
    };
    /**
     * Add a FP32 tensor from float xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<float>& tensor) {
        return addTensor<float>(name, tensor, dai::TensorInfo::DataType::FP32);
    };
    /**
     * Add a FP64 tensor from double xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<double>& tensor) {
        return addTensor<double>(name, tensor, dai::TensorInfo::DataType::FP64);
    };
    /**
     * Add an INT8 tensor from int8 xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<std::int8_t>& tensor) {
        return addTensor<std::int8_t>(name, tensor, dai::TensorInfo::DataType::I8);
    };
    /**
     * Add a U8F tensor from uint8 xt::xarray data.
     */
    NNData& addTensor(const std::string& name, const xt::xarray<std::uint8_t>& tensor) {
        return addTensor<std::uint8_t>(name, tensor, dai::TensorInfo::DataType::U8F);
    };

    /**
     * @brief Add a tensor to this NNData object.
     * The provided array is stored as a 1xN tensor where N is the length of the array.
     *
     * @param name: Name of the tensor
     * @param data: array
     * @param order: Storage order of the tensor
     * @return NNData&: reference to this object
     */
    template <typename _Ty = double>
    NNData& addTensor(const std::string& name, const std::vector<_Ty>& data, TensorInfo::StorageOrder order) {
        return addTensor<_Ty>(name, xt::adapt(data, std::vector<size_t>{1, data.size()}), order);
    };

    /**
     * @brief Add a tensor to this NNData object.
     * Implicitly adds a TensorInfo::DataType
     *
     * @param name: Name of the tensor
     * @param data: array
     * @param order: Storage order of the tensor
     * @return NNData&: reference to this object
     */

    template <typename _Ty = double>
    NNData& addTensor(const std::string& name, const xt::xarray<_Ty>& data, TensorInfo::StorageOrder order) {
        auto dataType = std::is_integral<_Ty>::value ? dai::TensorInfo::DataType::U8F : dai::TensorInfo::DataType::FP16;
        return addTensor<_Ty>(name, data, dataType, order);
    };

    /**
     * @brief Add a tensor to this NNData object. The storage order is picked based on the number of dimensions of the tensor.
     * Float values are converted to FP16 and integers are cast to bytes.
     *
     * @param name: Name of the tensor
     * @param tensor: tensor
     * @return NNData&: reference to this object
     */
    template <typename _Ty = double>
    NNData& addTensor(const std::string& name, const xt::xarray<_Ty>& tensor, dai::TensorInfo::DataType dataType) {
        TensorInfo::StorageOrder order;
        switch(tensor.shape().size()) {
            case 1:
                order = TensorInfo::StorageOrder::C;
                break;
            case 2:
                order = TensorInfo::StorageOrder::NC;
                break;
            case 3:
                order = TensorInfo::StorageOrder::CHW;
                break;
            case 4:
                order = TensorInfo::StorageOrder::NCHW;
                break;
            default:
                throw std::runtime_error("Unsupported tensor shape. Only 1D, 2D, 3D and 4D tensors are supported");
        }
        return addTensor(name, tensor, dataType, order);
    }

    /**
     * @brief Add a tensor to this NNData object. The storage order is picked based on the number of dimensions of the tensor.
     * Float values are converted to FP16 and integers are cast to bytes.
     *
     * @param name: Name of the tensor
     * @param tensor: tensor
     * @param order: Storage order of the tensor
     * @return NNData&: reference to this object
     */
    template <typename _Ty = double>
    NNData& addTensor(const std::string& name, const xt::xarray<_Ty>& tensor, dai::TensorInfo::DataType dataType, const TensorInfo::StorageOrder order) {
        static_assert(std::is_integral<_Ty>::value || std::is_floating_point<_Ty>::value, "Tensor type needs to be integral or floating point");
        // if(dataType==dai::TensorInfo::DataType::FP32) std::cout<<"FP32\n";
        // else if(dataType==dai::TensorInfo::DataType::FP16) std::cout<<"FP16\n";
        // else if(dataType==dai::TensorInfo::DataType::INT) std::cout<<"INT\n";
        // else if(dataType==dai::TensorInfo::DataType::I8) std::cout<<"I8\n";
        // else if(dataType==dai::TensorInfo::DataType::U8F) std::cout<<"U8F\n";
        // else if(dataType==dai::TensorInfo::DataType::FP64) std::cout<<"FP64\n";
        // else std::cout<<"Unsupported type\n";

        // Check if data is vector type of data
        if(std::dynamic_pointer_cast<VectorMemory>(data) == nullptr) {
            auto prev = std::vector<uint8_t>(data->getData().begin(), data->getData().end());
            data = std::make_shared<VectorMemory>(std::move(prev));
        }
        auto vecData = std::dynamic_pointer_cast<VectorMemory>(data);

        // Get size in bytes of the converted tensor data, u8 for integral and fp16 for floating point
        // const size_t sConvertedData = std::is_integral<_Ty>::value ? tensor.size() : 2 * tensor.size();
        size_t sConvertedData = tensor.size();
        switch(dataType) {
            case dai::TensorInfo::DataType::FP64:
                sConvertedData *= 8;
                break;
            case dai::TensorInfo::DataType::FP32:
            case dai::TensorInfo::DataType::INT:
                sConvertedData *= 4;
                break;
            case dai::TensorInfo::DataType::FP16:
                sConvertedData *= 2;
                break;
            case dai::TensorInfo::DataType::U8F:
            case dai::TensorInfo::DataType::I8:
                break;
        }

        // Append bytes so that each new tensor is DATA_ALIGNMENT aligned
        size_t remainder = std::distance(vecData->begin(), vecData->end()) % DATA_ALIGNMENT;
        if(remainder > 0) {
            vecData->insert(vecData->end(), DATA_ALIGNMENT - remainder, 0);
        }

        // Then get offset to beginning of data
        size_t offset = std::distance(vecData->begin(), vecData->end());

        // Reserve space
        vecData->resize(offset + sConvertedData);

        // Convert data to appropriate data type and write to data
        if(dataType == dai::TensorInfo::DataType::I8) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                vecData->data()[i + offset] = (int8_t)tensor.data()[i];
            }
        } else if(dataType == dai::TensorInfo::DataType::FP16) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(uint16_t*)(&vecData->data()[2 * i + offset]) = fp32_to_fp16(tensor.data()[i]);
            }
        } else if(dataType == dai::TensorInfo::DataType::FP32) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(float*)(&vecData->data()[4 * i + offset]) = (float)tensor.data()[i];
            }
        } else if(dataType == dai::TensorInfo::DataType::INT) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(int32_t*)(&vecData->data()[4 * i + offset]) = (int32_t)tensor.data()[i];
            }
        } else if(dataType == dai::TensorInfo::DataType::U8F) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                vecData->data()[i + offset] = (uint8_t)tensor.data()[i];
            }
        } else if(dataType == dai::TensorInfo::DataType::FP64) {
            for(uint32_t i = 0; i < tensor.size(); i++) {
                *(double*)(&vecData->data()[8 * i + offset]) = (double)tensor.data()[i];
            }
        }

        // Add entry in tensors
        TensorInfo info;
        info.name = name;
        info.offset = static_cast<unsigned int>(offset);
        info.dataType = dataType;
        info.numDimensions = tensor.dimension();
        info.order = order;
        for(uint32_t i = 0; i < tensor.dimension(); i++) {
            info.dims.push_back(tensor.shape()[i]);
            info.strides.push_back(tensor.strides()[i] * info.getDataTypeSize());
        }

        // Validate storage order - past this point, the tensor shape and storage order should be correct
        try {
            info.validateStorageOrder();
        } catch(...) {
            vecData->resize(offset);  // Resize vector back to its size prior to adding tensor
            throw;
        }

        tensors.push_back(info);
        return *this;
    }

    /**
     * Convenience function to retrieve values from a tensor
     * @returns xt::xarray<_Ty> tensor
     */
    template <typename _Ty>
    xt::xarray<_Ty> getTensor(const std::string& name, bool dequantize = false) {
        const auto it = std::find_if(tensors.begin(), tensors.end(), [&name](const TensorInfo& ti) { return ti.name == name; });

        if(it == tensors.end()) throw std::runtime_error("Tensor does not exist");

        std::vector<size_t> dims;
        for(const auto v : it->dims) {
            dims.push_back(v);
        }

        std::vector<size_t> strides;
        for(const auto v : it->strides) {
            strides.push_back(v);
        }

        xt::xarray<_Ty, xt::layout_type::row_major> tensor(dims);

        switch(it->dataType) {
            case TensorInfo::DataType::U8F:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = data->getData().data()[it->offset + i];
                }
                break;
            case TensorInfo::DataType::I8:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = reinterpret_cast<int8_t*>(data->getData().data())[it->offset + i];
                }
                break;
            case TensorInfo::DataType::INT:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = reinterpret_cast<int32_t*>(data->getData().data())[it->offset / sizeof(int32_t) + i];
                }
                break;
            case TensorInfo::DataType::FP16:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = fp16_to_fp32(reinterpret_cast<uint16_t*>(data->getData().data())[it->offset / sizeof(uint16_t) + i]);
                }
                break;
            case TensorInfo::DataType::FP32:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = reinterpret_cast<float_t*>(data->getData().data())[it->offset / sizeof(float_t) + i];
                }
                break;
            case TensorInfo::DataType::FP64:
                for(uint32_t i = 0; i < tensor.size(); i++) {
                    tensor.data()[i] = reinterpret_cast<double_t*>(data->getData().data())[it->offset / sizeof(double_t) + i];
                }
                break;
        }
        if(dequantize) {
            if(it->quantization) {
                tensor = (tensor - it->qpZp) * it->qpScale;
            }
        }
        return tensor;
    }

    /**
     * Convenience function to retrieve values from a tensor
     * @returns xt::xarray<_Ty> tensor
     */
    template <typename _Ty>
    xt::xarray<_Ty> getTensor(const std::string& name, TensorInfo::StorageOrder order, bool dequantize = false) {
        // Get tensor
        xt::xarray<_Ty> tensor = getTensor<_Ty>(name, dequantize);

        // Change storage order
        const auto storageit = std::find_if(tensors.begin(), tensors.end(), [&name](const TensorInfo& ti) { return ti.name == name; });
        TensorInfo::StorageOrder from = storageit->order;
        TensorInfo::StorageOrder to = order;
        changeStorageOrder(tensor, from, to);
        return tensor;
    }

    template <typename _Ty>
    /**
     * Change tensor storage order in-place.
     */
    void changeStorageOrder(xt::xarray<_Ty>& array, TensorInfo::StorageOrder from, TensorInfo::StorageOrder to) {
        // Convert storage order to vector
        auto order2vec = [](TensorInfo::StorageOrder order) {
            size_t ord = static_cast<size_t>(order);
            std::vector<size_t> vec;
            while(ord > 0) {
                vec.push_back(ord % 16 - 1);  // 16 because order values are base 16
                ord /= 16;
            }
            std::reverse(vec.begin(), vec.end());
            return vec;
        };
        std::vector<size_t> fromOrder = order2vec(from);
        std::vector<size_t> toOrder = order2vec(to);

        // Just permute dimensions
        if(fromOrder.size() == toOrder.size()) {
            std::vector<size_t> permute;
            for(size_t i = 0; i < toOrder.size(); ++i) {
                auto it = std::find(fromOrder.begin(), fromOrder.end(), toOrder[i]);
                if(it == fromOrder.end()) {
                    throw std::runtime_error("Cannot change storage order. Dimension not found [Permute]");
                }
                size_t dim = std::distance(fromOrder.begin(), it);
                permute.push_back(dim);
            }
            array = xt::transpose(array, permute);
        }

        // Expand and permute
        if(fromOrder.size() < toOrder.size()) {
            // Expand dimensions
            std::vector<size_t> expand;
            std::vector<size_t> fromOrderExpanded = fromOrder;
            for(size_t i = 0; i < toOrder.size(); ++i) {
                auto it = std::find(fromOrder.begin(), fromOrder.end(), toOrder[i]);
                if(it == fromOrder.end()) {
                    expand.push_back(fromOrderExpanded.size());
                    fromOrderExpanded.push_back(toOrder[i]);
                }
            }
            for(size_t i = 0; i < expand.size(); ++i) {
                array = xt::expand_dims(array, expand[i]);
            }

            // Permute
            std::vector<size_t> permute;
            for(size_t i = 0; i < toOrder.size(); ++i) {
                auto it = std::find(fromOrderExpanded.begin(), fromOrderExpanded.end(), toOrder[i]);
                if(it == fromOrderExpanded.end()) {
                    throw std::runtime_error("Cannot change storage order. Dimension not found [Expand and Permute]");
                }
                size_t dim = std::distance(fromOrderExpanded.begin(), it);
                permute.push_back(dim);
            }
            array = xt::transpose(array, permute);
        }

        // Squeeze and permute
        if(fromOrder.size() > toOrder.size()) {
            // Squeeze extra dimensions
            std::vector<size_t> squeeze;
            std::vector<size_t> fromOrderSqueezed;
            for(size_t i = 0; i < fromOrder.size(); ++i) {
                auto it = std::find(toOrder.begin(), toOrder.end(), fromOrder[i]);
                if(it == toOrder.end()) {
                    if(array.shape()[i] != 1) {
                        throw std::runtime_error("Cannot change storage order. Squeeze dimension is not 1");
                    } else {
                        squeeze.push_back(i);
                    }
                } else {
                    fromOrderSqueezed.push_back(fromOrder[i]);
                }
            }
            for(size_t i = 0; i < squeeze.size(); ++i) {
                array = xt::squeeze(array, squeeze[i]);
            }

            // Permute
            std::vector<size_t> permute;
            for(size_t i = 0; i < toOrder.size(); ++i) {
                auto it = std::find(fromOrderSqueezed.begin(), fromOrderSqueezed.end(), toOrder[i]);
                if(it == fromOrderSqueezed.end()) {
                    throw std::runtime_error("Cannot change storage order. Dimension not found [Squeeze and Permute]");
                }
                size_t dim = std::distance(fromOrderSqueezed.begin(), it);
                permute.push_back(dim);
            }
            array = xt::transpose(array, permute);
        }
    }

    /**
     * Convenience function to retrieve values from the first tensor
     * @returns xt::xarray<_Ty> tensor
     */
    template <typename _Ty>
    xt::xarray<_Ty> getFirstTensor(bool dequantize = false) {
        if(!tensors.empty()) {
            return getTensor<_Ty>(tensors[0].name, dequantize);
        }

        return {};
    }

    /**
     * Convenience function to retrieve values from the first tensor
     * @returns xt::xarray<_Ty> tensor
     */
    template <typename _Ty>
    xt::xarray<_Ty> getFirstTensor(TensorInfo::StorageOrder order, bool dequantize = false) {
        if(!tensors.empty()) {
            return getTensor<_Ty>(tensors[0].name, order, dequantize);
        }

        return {};
    }

#endif
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::NNData;
    }

    DEPTHAI_SERIALIZE(NNData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, tensors, batchSize, transformation);
};

}  // namespace dai
