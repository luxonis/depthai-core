#pragma once

#include <ostream>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {

/**
 * Tracklet structure
 *
 * Contains tracklets from object tracker output.
 */
struct Tracklet {
    enum class TrackingStatus : std::int32_t {
        NEW,     /**< The object is newly added. */
        TRACKED, /**< The object is being tracked. */
        LOST,   /**< The object gets lost now. The object can be tracked again automatically(long term tracking) or by specifying detected object manually(short
                  term and zero term tracking). */
        REMOVED /**< The object is removed. */
    };
    /**
     * Tracked region of interest.
     */
    Rect roi;
    /**
     * Tracklet's ID.
     */
    std::int32_t id = 0;
    /**
     * Tracklet's label ID.
     */
    std::int32_t label = 0;
    /**
     * Number of frames it is being tracked for.
     */
    std::int32_t age = 0;
    /**
     * Status of tracklet.
     */
    TrackingStatus status = TrackingStatus::LOST;

    /**
     * Image detection that is tracked.
     */
    ImgDetection srcImgDetection;
    /**
     * Spatial coordinates of tracklet.
     */
    Point3f spatialCoordinates;
    DEPTHAI_SERIALIZE(Tracklet, roi, id, label, age, status, srcImgDetection, spatialCoordinates);
};

/**
 * Tracklets message. Carries object tracking information.
 */
class Tracklets : public Buffer {
   public:
    /**
     * Construct Tracklets message.
     */
    Tracklets() = default;
    virtual ~Tracklets() = default;

    /**
     * Retrieve data for Tracklets.
     * @returns Vector of object tracker data, carrying tracking information.
     */
    std::vector<Tracklet> tracklets;
};

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::Tracklet::TrackingStatus& status) {
    switch(status) {
        case dai::Tracklet::TrackingStatus::NEW:
            out << "NEW";
            break;
        case dai::Tracklet::TrackingStatus::TRACKED:
            out << "TRACKED";
            break;
        case dai::Tracklet::TrackingStatus::LOST:
            out << "LOST";
            break;
        case dai::Tracklet::TrackingStatus::REMOVED:
            out << "REMOVED";
            break;
    }
    return out;
}