#pragma once

#include <ostream>
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
     * Retrieve data for Tracklets.
     * @returns Vector of object tracker data, carrying tracking information.
     */
    std::vector<Tracklet>& tracklets;
};

inline std::ostream& operator<<(std::ostream& out, const Tracklet::TrackingStatus& status) {
    switch(status) {
        case Tracklet::TrackingStatus::NEW:
            out << "NEW";
            break;
        case Tracklet::TrackingStatus::TRACKED:
            out << "TRACKED";
            break;
        case Tracklet::TrackingStatus::LOST:
            out << "LOST";
            break;
        case Tracklet::TrackingStatus::REMOVED:
            out << "REMOVED";
            break;
    }
    return out;
}

}  // namespace dai
