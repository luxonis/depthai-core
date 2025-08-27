#pragma once

// std
#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

/**
 * Keypoints message. Carries a normalized keypoint 3D position, confidence score, and label.
 */
class Keypoints : public Buffer, public ProtoSerializable {
   private:
    KeypointsList keypointsList;

   public:
    /**
     * Constructs Keypoints message.
     */
    Keypoints() = default;
    virtual ~Keypoints() = default;

    // Keypoints
    std::optional<ImgTransformation> transformation;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Keypoints;
    };

    /*
     * Sets the keypoints list.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */

    void setKeypoints(const std::vector<Keypoint>& keypoints);

    void setKeypoints(const std::vector<Point3f>& keypoints);

    void setKeypoints(const std::vector<Point2f>& keypoints);

    void setKeypoints(const std::vector<Keypoint>& keypoints, const std::vector<std::array<uint32_t, 2>>& edges);

    void setEdges(const std::vector<std::array<uint32_t, 2>>& edges);

    /*
     * Returns a list of Keypoint objects.
     */
    std::vector<Keypoint> getKeypoints() const;

    /*
     * Returns a list of edges, each edge is a pair of indices.
     */
    std::vector<std::array<uint32_t, 2>> getEdges() const;

    /*
     * Returns a list of the coordinates of the keypoints as Point3f objects.
     */
    std::vector<Point3f> getCoordinates3f() const;

    /*
     * Returns a list of the coordinates of the keypoints as Point2f objects.
     */
    std::vector<Point2f> getCoordinates2f() const;

    /*
     * Returns a list of keypoint labels.
     */
    std::vector<std::string> getLabels() const;

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