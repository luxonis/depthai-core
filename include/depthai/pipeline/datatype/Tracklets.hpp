#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTracklets.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Tracklets message. Carries object tracking information.
 */
class Tracklets : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawTracklets& rawdata;

   public:
    /**
     * Construct Tracklets message.
     */
    Tracklets();
    explicit Tracklets(std::shared_ptr<RawTracklets> ptr);
    virtual ~Tracklets() = default;

    /**
     * Retrieve configuration data for Tracklets.
     * @returns Vector of object tracker data, carrying tracking information.
     */
    std::vector<Tracklet>& getTracklets() const;

    std::vector<Tracklet>& tracklets;
};

}  // namespace dai
