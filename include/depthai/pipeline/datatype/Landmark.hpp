#pragma once

#include <vector>

#include "DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {
struct Landmark {
    /**
     * The ID of the Landmark
     */
    int id;

    /**
     * The size of the landmark in meters
     */
    double size;

    /**
     * The pose of the landmark reletive to base_frame (the camera)
     */
    dai::TransformData pose;
};


class Landmarks : public Buffer {
    public:
        Landmarks() = default;
        virtual ~Landmarks() = default;

    public:
    std::vector<Landmark> landmarks;
    DEPTHAI_SERIALIZE(Landmarks, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, landmarks);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Landmarks;
    }

};

}
