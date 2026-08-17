#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace beta {

/**
 * Cluster of 2D points. Serialized value type contained by the Clusters message.
 */
struct Cluster {
    /**
     * Label of the cluster.
     */
    std::int32_t label = 0;

    /**
     * Points in the cluster.
     */
    std::vector<Point2f> points;

    DEPTHAI_SERIALIZE(Cluster, label, points);
};

/**
 * Clusters message. Carries clusters of 2D points, each cluster with an integer label.
 *
 * Parsers emit clusters with sequential labels starting at 0 and point image coordinates
 * normalized to [0, 1]. Clusters may be empty, e.g. lanes without enough detected points.
 */
class Clusters : public Buffer, public TransformableCRTP<Clusters> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * Remaps the point image coordinates of every cluster from the source transformation
     * carried by this message into the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Clusters>;

    /**
     * Construct Clusters message.
     */
    Clusters() = default;
    ~Clusters() override;

    /**
     * Detected clusters of points.
     */
    std::vector<Cluster> clusters;

    /**
     * Returns a new Clusters message with the cluster point image coordinates remapped from
     * this message's transformation into the target transformation.
     *
     * @param target Target image transformation.
     * @throws std::runtime_error if this message carries no transformation metadata.
     */
    Clusters transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgAnnotations visualization with each cluster drawn as points in a distinct
     * color sampled from a rainbow colormap.
     *
     * @throws std::runtime_error if the message contains more than 255 clusters.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Clusters;
    }

    DEPTHAI_SERIALIZE(Clusters, sequenceNum, ts, tsDevice, tsSystem, transformation, clusters);
};

}  // namespace beta
}  // namespace dai
