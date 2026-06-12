#pragma once

#include <ostream>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

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
    /**
     * Estimated 3D velocity of the tracklet in m/s.
     * nullopt when spatial data is unavailable.
     */
    std::optional<Point3f> velocity;
    /**
     * Magnitude of the estimated 3D velocity in m/s.
     * nullopt when spatial data is unavailable.
     */
    std::optional<float> speed;

    /**
     * Transforms the tracklet to the target image transformation.
     * @param source Source image transformation.
     * @param target Target image transformation.
     * @param lengthUnit Length unit used by this tracklet's spatial coordinates.
     */
    void transform(const ImgTransformation& source, const ImgTransformation& target, LengthUnit lengthUnit = LengthUnit::MILLIMETER);

    DEPTHAI_SERIALIZE(Tracklet, roi, id, label, age, status, srcImgDetection, spatialCoordinates, velocity, speed);
};

/**
 * Tracklets message. Carries object tracking information.
 */
class Tracklets : public Buffer, public TransformableCRTP<Tracklets> {
   protected:
    /**
     * Internal transform hook used by transformTo() to apply Tracklets-specific transformation logic.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    friend class TransformableCRTP<Tracklets>;
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Transformable::getTransformation;
    using Transformable::setTransformation;
    using Transformable::transformation;

    /**
     * Measurement unit used by all tracklets' `spatialCoordinates` in this list.
     */
    LengthUnit unit = LengthUnit::MILLIMETER;

    /**
     * Construct Tracklets message.
     */
    Tracklets() = default;
    virtual ~Tracklets();

    /**
     * Retrieve data for Tracklets.
     * @returns Vector of object tracker data, carrying tracking information.
     */
    std::vector<Tracklet> tracklets;

    /**
     * Returns a new Tracklets message with the tracklets transformed into the target image transformation.
     *
     * For each tracklet, the bounding box is assumed to lie on a plane parallel to the image plane at depth `tracklet.spatialCoordinates.z` (that is, all
     * four bounding-box corners are projected using the same depth value). The transformed corners are then fit with the smallest enclosing rotated rectangle
     * to preserve rectangularity.
     *
     * @param target Target image transformation.
     */
    Tracklets transformTo(const ImgTransformation& target);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(Tracklets, tracklets, transformation, ts, tsDevice, sequenceNum, unit);
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
