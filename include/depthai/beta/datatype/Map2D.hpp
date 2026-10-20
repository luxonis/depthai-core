#pragma once

#include <cstddef>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
namespace beta {

/**
 * Map2D message. Carries a dense 2D map of 32-bit floats, such as a depth map, a density map or a
 * heat map, together with image transformation metadata.
 *
 * The map values are stored row-major in the buffer payload; the map dimensions are carried in the
 * serialized metadata.
 */
class Map2D : public Buffer, public TransformableCRTP<Map2D> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * The map carries no remappable spatial data, so only the transformation metadata is replaced
     * with the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   private:
    size_t width = 0;
    size_t height = 0;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Map2D>;

    /**
     * Construct Map2D message.
     */
    Map2D();

    /**
     * Construct Map2D message with the given map values and dimensions.
     *
     * @param map Map values in row-major order, of size width * height.
     * @param width Map width in values per row.
     * @param height Map height in rows.
     * @throws std::runtime_error if the map size does not equal width * height.
     */
    Map2D(const std::vector<float>& map, size_t width, size_t height);
    ~Map2D() override;

    /**
     * Sets the 2D map. The values are copied into the buffer payload.
     *
     * @param map Map values in row-major order, of size width * height.
     * @param width Map width in values per row.
     * @param height Map height in rows.
     * @throws std::runtime_error if the map size does not equal width * height.
     */
    void setMap(const std::vector<float>& map, size_t width, size_t height);

    /**
     * Sets the 2D map from a float span without an extra temporary vector. The values are copied
     * into the buffer payload.
     *
     * @param map Map values in row-major order, of size width * height.
     * @param width Map width in values per row.
     * @param height Map height in rows.
     * @throws std::runtime_error if the map size does not equal width * height.
     */
    void setMap(span<const float> map, size_t width, size_t height);

    /**
     * Returns a copy of the 2D map values in row-major order. If no map is set, returns an empty
     * vector.
     */
    std::vector<float> getMap() const;

    /**
     * Returns the width of the 2D map.
     */
    size_t getWidth() const;

    /**
     * Returns the height of the 2D map.
     */
    size_t getHeight() const;

    /**
     * Returns a new Map2D message with the transformation metadata replaced by the target
     * transformation. The map values and dimensions are unchanged.
     *
     * @param target Target image transformation.
     * @throws std::runtime_error if this message carries no transformation metadata.
     */
    Map2D transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgFrame visualization of the map colored with a plasma colormap.
     *
     * When any map value is below 1 the values are scaled by 255, so maps normalized to [0, 1]
     * use the full colormap range. The values are then truncated to 8-bit indices into the
     * colormap and emitted as an interleaved BGR frame.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Map2D;
    }

    DEPTHAI_SERIALIZE(Map2D, sequenceNum, ts, tsDevice, tsSystem, transformation, width, height);
};

}  // namespace beta
}  // namespace dai
