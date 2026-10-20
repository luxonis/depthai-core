#pragma once

#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {
namespace beta {

/**
 * Keypoints message. Streamable wrapper around the native dai::KeypointsList, carrying 2D or 3D
 * keypoints together with optional skeleton edges connecting them.
 *
 * Keypoint image coordinates are normalized to [0, 1] by the keypoint parsers. 2D keypoints carry
 * a z coordinate of 0.
 */
class Keypoints : public Buffer, public TransformableCRTP<Keypoints> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * Remaps the keypoint image coordinates from the source transformation carried by this
     * message into the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Keypoints>;

    /**
     * Construct Keypoints message.
     */
    Keypoints() = default;
    ~Keypoints() override;

    /**
     * Native keypoints list carrying the keypoints and the skeleton edges connecting them.
     */
    KeypointsList keypointsList;

    /**
     * Returns the keypoints.
     */
    std::vector<Keypoint> getKeypoints() const;

    /**
     * Sets the keypoints.
     *
     * @param keypoints Keypoints to set.
     * @note This clears any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint>& keypoints);

    /**
     * Sets the keypoints together with the skeleton edges connecting them.
     *
     * @param keypoints Keypoints to set.
     * @param edges Pairs of keypoint indices to connect. Example: {{0, 1}, {1, 2}} connects
     *              keypoint 0 to keypoint 1 and keypoint 1 to keypoint 2.
     * @throws std::invalid_argument if an edge index is out of range or an edge is a self-loop.
     */
    void setKeypoints(const std::vector<Keypoint>& keypoints, const std::vector<Edge>& edges);

    /**
     * Returns the skeleton edges as pairs of keypoint indices.
     */
    std::vector<Edge> getEdges() const;

    /**
     * Sets the skeleton edges.
     *
     * @param edges Pairs of keypoint indices to connect.
     * @throws std::invalid_argument if an edge index is out of range or an edge is a self-loop.
     */
    void setEdges(const std::vector<Edge>& edges);

    /**
     * Returns the 2D image coordinates of the keypoints, dropping the z axis values.
     */
    std::vector<Point2f> getPoints2f() const;

    /**
     * Returns the 3D image coordinates of the keypoints. 2D keypoints carry a z coordinate of 0.
     */
    std::vector<Point3f> getPoints3f() const;

    /**
     * Returns a new Keypoints message with the keypoint image coordinates remapped from this
     * message's transformation into the target transformation.
     *
     * @param target Target image transformation.
     * @throws std::runtime_error if this message carries no transformation metadata.
     */
    Keypoints transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgAnnotations visualization with the keypoints drawn as points and the skeleton
     * edges drawn as lines.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Keypoints;
    }

    DEPTHAI_SERIALIZE(Keypoints, sequenceNum, ts, tsDevice, tsSystem, transformation, keypointsList);
};

}  // namespace beta
}  // namespace dai
