
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

#include "../../utility/ProtoSerialize.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImageAnnotations.pb.h"
    #include "utility/ProtoSerializable.hpp"
#endif

namespace dai {
#ifdef DEPTHAI_ENABLE_PROTOBUF
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgAnnotations* daiAnnotations) {
    auto imageAnnotations = std::make_unique<proto::image_annotations::ImageAnnotations>();

    imageAnnotations->set_sequencenum(daiAnnotations->sequenceNum);
    proto::common::Timestamp* ts = imageAnnotations->mutable_ts();
    ts->set_sec(daiAnnotations->ts.sec);
    ts->set_nsec(daiAnnotations->ts.nsec);
    proto::common::Timestamp* tsDevice = imageAnnotations->mutable_tsdevice();
    tsDevice->set_sec(daiAnnotations->tsDevice.sec);
    tsDevice->set_nsec(daiAnnotations->tsDevice.nsec);

    for(const auto& annotation : daiAnnotations->annotations) {
        proto::image_annotations::ImageAnnotation* imageAnnotation = imageAnnotations->add_annotations();
        for(const auto& circle : annotation.circles) {
            proto::image_annotations::CircleAnnotation* circleAnnotation = imageAnnotation->add_circles();
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
            proto::image_annotations::PointsAnnotation* pointsAnnotation = imageAnnotation->add_points();
            PointsAnnotationType type = points.type;
            pointsAnnotation->set_type(static_cast<proto::image_annotations::PointsAnnotationType>(type));
            for(const auto& point : points.points) {
                proto::common::Point2f* protoPoint = pointsAnnotation->add_points();
                protoPoint->set_x(point.x);
                protoPoint->set_y(point.y);
            }
            pointsAnnotation->mutable_outlinecolor()->set_r(points.outlineColor.r);
            pointsAnnotation->mutable_outlinecolor()->set_g(points.outlineColor.g);
            pointsAnnotation->mutable_outlinecolor()->set_b(points.outlineColor.b);
            pointsAnnotation->mutable_outlinecolor()->set_a(points.outlineColor.a);
            for(const auto& color : points.outlineColors) {
                proto::common::Color* protoColor = pointsAnnotation->add_outlinecolors();
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
            proto::image_annotations::TextAnnotation* textAnnotation = imageAnnotation->add_texts();
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

ProtoSerializable::SchemaPair ImgAnnotations::serializeSchema() const {
    return utility::serializeSchema(getProtoMessage(this));
}

std::vector<std::uint8_t> ImgAnnotations::serializeProto() const {
    return utility::serializeProto(getProtoMessage(this));
}

#endif

}  // namespace dai
