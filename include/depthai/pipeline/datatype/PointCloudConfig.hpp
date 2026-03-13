#pragma once

#include <array>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * PointCloudConfig message. Carries point cloud output settings.
 */
class PointCloudConfig : public Buffer {
    // false = filter to valid (z > 0) points only (default)
    // true  = keep all width*height points (organized)
    bool organized = false;

    std::array<std::array<float, 4>, 4> transformationMatrix = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};

    LengthUnit lengthUnit = LengthUnit::MILLIMETER;

   public:
    PointCloudConfig() = default;
    virtual ~PointCloudConfig();

    /**
     * Retrieve whether the point cloud is organized (all width*height points kept).
     * @returns true if all width*height points are output, false if only valid (z > 0) points are kept
     */
    bool getOrganized() const;

    /**
     * Retrieve transformation matrix applied to every output point.
     * @returns 4x4 column-major transformation matrix (identity by default)
     */
    std::array<std::array<float, 4>, 4> getTransformationMatrix() const;

    /**
     * Retrieve the length unit used for output point coordinates.
     */
    LengthUnit getLengthUnit() const;

    /**
     * Enable or disable organized point cloud output.
     * When true all width*height points are kept; when false only points with z > 0 are emitted.
     */
    PointCloudConfig& setOrganized(bool enable);

    /**
     * Set a 4x4 transformation matrix applied to every output point.
     * Default is the identity matrix.
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix);

    /**
     * Convenience overload: set a 3x3 rotation matrix (translation set to zero).
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix);

    /**
     * Set the length unit for output point coordinates.
     */
    PointCloudConfig& setLengthUnit(LengthUnit unit);

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PointCloudConfig;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(PointCloudConfig, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, organized, transformationMatrix, lengthUnit);
};

}  // namespace dai
