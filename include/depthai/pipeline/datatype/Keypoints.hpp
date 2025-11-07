#pragma once

// std
#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

/**
 * Keypoints message. Carries a normalized keypoint 3D position, confidence score and label index.
 */
class Keypoints : public Buffer, public ProtoSerializable {
   private:
    KeypointsList keypointsList;

   public:
    Keypoints() = default;
    virtual ~Keypoints() = default;
    std::optional<ImgTransformation> transformation;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Sets the keypoints.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint>& keypoints);

    /**
     * Sets the keypoints from a vector of 3D points.
     * @param keypoints vector of Point3f objects to set as keypoints.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point3f>& keypoints);

    /**
     * Sets the keypoints from a vector of 2D points.
     * @param keypoints vector of Point2f objects to set as keypoints.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point2f>& keypoints);

    /**
     * Sets the keypoints list along with edges.
     * @param keypoints list of Keypoint objects to set.
     * @param edges list of edges, each edge is a pair of indices into the key
     */
    void setKeypoints(const std::vector<Keypoint>& keypoints, const std::vector<Edge>& edges);

    /**
     * Sets the edges between the saved keypoints.
     * @param edges list of valid edges.
     */
    void setEdges(const std::vector<Edge>& edges);

    /*
     * Returns a list of Keypoint objects.
     */
    std::vector<Keypoint> getKeypoints() const;

    /*
     * Returns a list of edges, each edge is a pair of indices.
     */
    std::vector<Edge> getEdges() const;

    /*
     * Returns a list of the coordinates of the keypoints as Point3f objects.
     */
    std::vector<Point3f> getCoordinates3f() const;

    /*
     * Returns a list of the coordinates of the keypoints as Point2f objects.
     */
    std::vector<Point2f> getCoordinates2f() const;

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */

    std::vector<std::uint8_t> serializeProto(bool = false) const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    ProtoSerializable::SchemaPair serializeSchema() const override;

#endif

    DEPTHAI_SERIALIZE(Keypoints, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, keypointsList, transformation);
};

}  // namespace dai