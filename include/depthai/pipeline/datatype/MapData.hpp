#pragma once

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * RGBD message. Carries RGB and Depth frames.
 */
class MapData : public Buffer {
   public:
    /**
     * Construct RGBD message.
     */
    MapData() = default;

    virtual ~MapData() = default;

    dai::ImgFrame map;
    float minX;
    float minY;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::MapData;
    };
    DEPTHAI_SERIALIZE(MapData, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, map, minX, minY);
};

}  // namespace dai
