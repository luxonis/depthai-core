#pragma once

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * MapData message. Carries grid map data and minX/minY messages to help place it in 3D space.
 */
class MapData : public Buffer {
   public:
    MapData() = default;

    virtual ~MapData();

    dai::ImgFrame map;
    float minX = 0.0;
    float minY = 0.0;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    #ifndef DEPTHAI_MESSAGES_RVC2
    DEPTHAI_SERIALIZE(MapData, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, Buffer::sequenceNum, map, minX, minY);
    #else
    DEPTHAI_SERIALIZE(MapData, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, map, minX, minY);
    #endif
};

}  // namespace dai
