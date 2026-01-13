#pragma once
#include "depthai/common/Color.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {
struct CircleAnnotation {
    Point2f position;
    float diameter = 0.0f;
    float thickness = 1.0f;
    Color fillColor;
    Color outlineColor;
};

DEPTHAI_SERIALIZE_EXT(CircleAnnotation, position, diameter, thickness, fillColor, outlineColor);

enum class PointsAnnotationType : std::uint8_t { UNKNOWN = 0, POINTS = 1, LINE_LOOP = 2, LINE_STRIP = 3, LINE_LIST = 4 };

struct PointsAnnotation {
    PointsAnnotationType type = PointsAnnotationType::POINTS;
    std::vector<Point2f> points;
    Color outlineColor;
    std::vector<Color> outlineColors;
    Color fillColor;
    float thickness = 1.0f;
};

DEPTHAI_SERIALIZE_EXT(PointsAnnotation, type, points, outlineColor, outlineColors, fillColor, thickness);

struct TextAnnotation {
    Point2f position;
    std::string text;
    float fontSize = 12.0f;
    Color textColor;
    Color backgroundColor;
};
DEPTHAI_SERIALIZE_EXT(TextAnnotation, position, text, fontSize, textColor, backgroundColor);
struct ImgAnnotation {
    std::vector<CircleAnnotation> circles;
    std::vector<PointsAnnotation> points;
    std::vector<TextAnnotation> texts;
};

DEPTHAI_SERIALIZE_EXT(ImgAnnotation, circles, points, texts);

/**
 * ImgAnnotations message. Carries annotations for an image.
 */
class ImgAnnotations : public Buffer, public ProtoSerializable {
   public:
    /**
     * Construct ImgAnnotations message.
     */
    ImgAnnotations() = default;
    /**
     * Construct ImgAnnotations with a list of annotations.
     */
    explicit ImgAnnotations(std::vector<ImgAnnotation> annotations) : annotations(std::move(annotations)) {}

    virtual ~ImgAnnotations();

    /// Transform
    std::vector<ImgAnnotation> annotations;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ImgAnnotations;
    }

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

    DEPTHAI_SERIALIZE(ImgAnnotations, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, annotations);
};

}  // namespace dai
