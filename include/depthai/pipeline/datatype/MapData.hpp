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

    float minX = 0.0;
    float minY = 0.0;
    std::shared_ptr<dai::ImgFrame> getMap();
    void setMap(std::shared_ptr<ImgFrame> map);

    private:
    std::shared_ptr<dai::ADatatype> map;
public:
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DEPTHAI_SERIALIZE(MapData, map, minX, minY, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai
