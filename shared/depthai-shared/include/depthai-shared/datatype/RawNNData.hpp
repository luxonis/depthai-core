#pragma once

#include "depthai-shared/common/TensorInfo.hpp"
#include "depthai-shared/common/Timestamp.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawNNData structure
struct RawNNData : public RawBuffer {
    // NNData data is in PoBuf
    std::vector<TensorInfo> tensors;
    unsigned int batchSize;

    // Related to input ImgFrame
    int64_t sequenceNum = 0;  // increments for each frame
    Timestamp ts = {};        // generation timestamp, synced to host time
    Timestamp tsDevice = {};  // generation timestamp, direct device monotonic clock

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::NNData;
    };

    DEPTHAI_SERIALIZE(RawNNData, tensors, batchSize, sequenceNum, ts, tsDevice);
};

}  // namespace dai
