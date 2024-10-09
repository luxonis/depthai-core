
#pragma once
#include "depthai/common/Point2f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/schemas/ImageAnnotations.pb.h"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {
struct Color {
    float r;
    float g;
    float b;
    float a;
};
DEPTHAI_SERIALIZE_EXT(Color, r, g, b, a);
struct CircleAnnotation {
    Point2f position;
    float diameter;
    float thickness;
    Color fillColor;
    Color outlineColor;
};

DEPTHAI_SERIALIZE_EXT(CircleAnnotation, position, diameter, thickness, fillColor, outlineColor);

enum class PointsAnnotationType : std::uint8_t { UNKNOWN = 0, POINTS = 1, LINE_LOOP = 2, LINE_STRIP = 4, LINE_LIST = 4 };

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
    ImageAnnotations();
    explicit ImageAnnotations(std::vector<ImageAnnotations> annotations);

    virtual ~ImageAnnotations() = default;

    /// Transform
    std::vector<ImageAnnotation> annotations;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageAnnotations;
    };
    std::unique_ptr<google::protobuf::Message> getProtoMessage() const override {
        auto imageAnnotations = std::make_unique<proto::ImageAnnotations>();

        imageAnnotations->set_sequencenum(this->sequenceNum);
        proto::Timestamp* ts = imageAnnotations->mutable_ts();
        ts->set_sec(this->ts.sec);
        ts->set_nsec(this->ts.nsec);
        proto::Timestamp* tsDevice = imageAnnotations->mutable_tsdevice();
        tsDevice->set_sec(this->tsDevice.sec);
        tsDevice->set_nsec(this->tsDevice.nsec);

        for(const auto& annotation : this->annotations) {
            proto::ImageAnnotation* imageAnnotation = imageAnnotations->add_annotations();
            for(const auto& circle : annotation.circles) {
                proto::CircleAnnotation* circleAnnotation = imageAnnotation->add_circles();
                circleAnnotation->mutable_position()->set_x(circle.position.x);
                circleAnnotation->mutable_position()->set_y(circle.position.y);
                circleAnnotation->set_diameter(circle.diameter);
                circleAnnotation->set_thickness(circle.thickness);
                circleAnnotation->mutable_fillcolor()->set_r(circle.fillColor.r);
                circleAnnotation->mutable_fillcolor()->set_g(circle.fillColor.g);
                circleAnnotation->mutable_fillcolor()->set_b(circle.fillColor.b);
                circleAnnotation->mutable_fillcolor()->set_a(circle.fillColor.a);
                circleAnnotation->mutable_outlinecolor()->set_r(circle.outlineColor.r);
                circleAnnotation->mutable_outlinecolor()->set_g(circle.outlineColor.g);
                circleAnnotation->mutable_outlinecolor()->set_b(circle.outlineColor.b);
                circleAnnotation->mutable_outlinecolor()->set_a(circle.outlineColor.a);
            }
            for(const auto& points : annotation.points) {
                proto::PointsAnnotation* pointsAnnotation = imageAnnotation->add_points();
                PointsAnnotationType type = points.type;
                pointsAnnotation->set_type(static_cast<proto::PointsAnnotationType>(type));
                for(const auto& point : points.points) {
                    proto::Point2f* protoPoint = pointsAnnotation->add_points();
                    protoPoint->set_x(point.x);
                    protoPoint->set_y(point.y);
                }
                pointsAnnotation->mutable_outlinecolor()->set_r(points.outlineColor.r);
                pointsAnnotation->mutable_outlinecolor()->set_g(points.outlineColor.g);
                pointsAnnotation->mutable_outlinecolor()->set_b(points.outlineColor.b);
                pointsAnnotation->mutable_outlinecolor()->set_a(points.outlineColor.a);
                for(const auto& color : points.outlineColors) {
                    proto::Color* protoColor = pointsAnnotation->add_outlinecolors();
                    protoColor->set_r(color.r);
                    protoColor->set_g(color.g);
                    protoColor->set_b(color.b);
                    protoColor->set_a(color.a);
                }
                pointsAnnotation->mutable_fillcolor()->set_r(points.fillColor.r);
                pointsAnnotation->mutable_fillcolor()->set_g(points.fillColor.g);
                pointsAnnotation->mutable_fillcolor()->set_b(points.fillColor.b);
                pointsAnnotation->mutable_fillcolor()->set_a(points.fillColor.a);
                pointsAnnotation->set_thickness(points.thickness);
            }
            for(const auto& text : annotation.texts) {
                proto::TextAnnotation* textAnnotation = imageAnnotation->add_texts();
                textAnnotation->mutable_position()->set_x(text.position.x);
                textAnnotation->mutable_position()->set_y(text.position.y);
                textAnnotation->set_text(text.text);
                textAnnotation->set_fontsize(text.fontSize);
                textAnnotation->mutable_textcolor()->set_r(text.textColor.r);
                textAnnotation->mutable_textcolor()->set_g(text.textColor.g);
                textAnnotation->mutable_textcolor()->set_b(text.textColor.b);
                textAnnotation->mutable_textcolor()->set_a(text.textColor.a);
                textAnnotation->mutable_backgroundcolor()->set_r(text.backgroundColor.r);
                textAnnotation->mutable_backgroundcolor()->set_g(text.backgroundColor.g);
                textAnnotation->mutable_backgroundcolor()->set_b(text.backgroundColor.b);
                textAnnotation->mutable_backgroundcolor()->set_a(text.backgroundColor.a);
            }
        }
        return imageAnnotations;
    }

    DEPTHAI_SERIALIZE(ImageAnnotations, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, annotations);
};

}  // namespace dai
