#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * PointCloudConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class PointCloudConfig : public Buffer {
    bool sparse = false;

    std::array<std::array<float, 4>, 4> transformationMatrix = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};

   public:
    /**
     * Construct PointCloudConfig message.
     */
    PointCloudConfig() = default;
    virtual ~PointCloudConfig() = default;

    /**
     * Retrieve sparse point cloud calculation status.
     * @returns true if sparse point cloud calculation is enabled, false otherwise
     */
    bool getSparse() const;

    /**
     * Retrieve transformation matrix for point cloud calculation.
     * @returns 4x4 transformation matrix
     */
    std::array<std::array<float, 4>, 4> getTransformationMatrix() const;

    /**
     * Enable or disable sparse point cloud calculation.
     * @param enable
     */
    PointCloudConfig& setSparse(bool enable);

    /**
     * Set 4x4 transformation matrix for point cloud calculation. Default is an identity matrix.
     * @param transformationMatrix
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix);

    /**
     * Set 3x3 transformation matrix for point cloud calculation. Default is an identity matrix.
     * @param transformationMatrix
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PointCloudConfig;
    };

    DEPTHAI_SERIALIZE(PointCloudConfig, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, sparse, transformationMatrix);
};

}  // namespace dai
