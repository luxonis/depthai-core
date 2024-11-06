
#pragma once
#include "depthai/common/Color.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {
struct CircleAnnotation {
    Point2f position;
    float diameter;
    float thickness;
    Color fillColor;
    Color outlineColor;
};

DEPTHAI_SERIALIZE_EXT(CircleAnnotation, position, diameter, thickness, fillColor, outlineColor);

enum class PointsAnnotationType : std::uint8_t { UNKNOWN = 0, POINTS = 1, LINE_LOOP = 2, LINE_STRIP = 3, LINE_LIST = 4 };

struct PointsAnnotation {
    PointsAnnotationType type;
    std::vector<Point2f> points;
    Color outlineColor;
    std::vector<Color> outlineColors;
    Color fillColor;
    float thickness;
};

DEPTHAI_SERIALIZE_EXT(PointsAnnotation, type, points, outlineColor, outlineColors, fillColor, thickness);

struct TextAnnotation {
    Point2f position;
    std::string text;
    float fontSize;
    Color textColor;
    Color backgroundColor;
};
DEPTHAI_SERIALIZE_EXT(TextAnnotation, position, text, fontSize, textColor, backgroundColor);
struct ImageAnnotation {
    std::vector<CircleAnnotation> circles;
    std::vector<PointsAnnotation> points;
    std::vector<TextAnnotation> texts;
};

DEPTHAI_SERIALIZE_EXT(ImageAnnotation, circles, points, texts);

/**
 * ImageAnnotations message. Carries annotations for an image.
 */
class ImageAnnotations : public Buffer, public utility::ProtoSerializable {
   public:
    /**
     * Construct ImageAnnotations message.
     */
    ImageAnnotations() = default;
    explicit ImageAnnotations(std::vector<ImageAnnotation> annotations) : annotations(std::move(annotations)) {}

    virtual ~ImageAnnotations() = default;

    /// Transform
    std::vector<ImageAnnotation> annotations;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageAnnotations;
    };

        /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto() const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    utility::ProtoSerializable::SchemaPair serializeSchema() const override;

    DEPTHAI_SERIALIZE(ImageAnnotations, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, annotations);
};

}  // namespace dai
