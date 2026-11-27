#pragma once

#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "OpticalFlowPacking.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace gate {

inline std::shared_ptr<MessageGroup> makeOpticalFlowOutput(
    const std::int16_t* forward, const std::int16_t* backward, const std::uint8_t* confidence, const ImgFrame& reference, const ImgFrame& target) {
    const auto width = target.getWidth();
    const auto height = target.getHeight();
    if(width <= 0 || height <= 0 || width > 1920 || height > 1080 || width % 128 || height % 32 || width != reference.getWidth()
       || height != reference.getHeight() || !confidence) {
        throw std::invalid_argument("Invalid native optical flow frame pair or confidence buffer");
    }
    const std::size_t pixels = static_cast<std::size_t>(width) * height;
    auto fwd = optical_flow::copyMotionVectors(forward, 2 * pixels);
    auto bwd = optical_flow::copyMotionVectors(backward, 2 * pixels);
    auto flow = std::make_shared<NNData>();
    flow->batchSize = 1;
    std::vector<std::uint8_t> bytes;
    bytes.reserve(pixels * (4 * sizeof(float) + 1) + 192);
    const auto append = [&](const char* name, TensorInfo::DataType type, unsigned int channels, const void* source, std::size_t elementSize) {
        bytes.resize((bytes.size() + 63) / 64 * 64);
        TensorInfo tensor{};
        tensor.name = name;
        tensor.dataType = type;
        tensor.order = TensorInfo::StorageOrder::HWC;
        tensor.numDimensions = 3;
        tensor.dims = {static_cast<unsigned int>(height), static_cast<unsigned int>(width), channels};
        tensor.strides = {static_cast<unsigned int>(width * channels * elementSize),
                          static_cast<unsigned int>(channels * elementSize),
                          static_cast<unsigned int>(elementSize)};
        tensor.offset = static_cast<unsigned int>(bytes.size());
        const auto count = pixels * channels * elementSize;
        bytes.resize(bytes.size() + count);
        std::memcpy(bytes.data() + tensor.offset, source, count);
        flow->tensors.push_back(tensor);
    };
    append("forward", TensorInfo::DataType::FP32, 2, fwd.data(), sizeof(float));
    append("backward", TensorInfo::DataType::FP32, 2, bwd.data(), sizeof(float));
    append("confidence", TensorInfo::DataType::U8F, 1, confidence, sizeof(std::uint8_t));
    flow->setData(std::move(bytes));
    flow->setBufferMetadataFrom(&target);
    auto ref = std::make_shared<Buffer>();
    auto cur = std::make_shared<Buffer>();
    ref->setBufferMetadataFrom(&reference);
    cur->setBufferMetadataFrom(&target);
    auto result = std::make_shared<MessageGroup>();
    result->add("flow", flow);
    result->add("reference", ref);
    result->add("target", cur);
    result->setBufferMetadataFrom(&target);
    return result;
}

}  // namespace gate
}  // namespace dai
