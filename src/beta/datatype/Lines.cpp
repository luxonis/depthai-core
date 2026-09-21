#include "depthai/beta/datatype/Lines.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

namespace dai {
namespace beta {

namespace {

// Visualization constants matching the source implementation in depthai-nodes.
const Color PRIMARY_COLOR{21.0f / 255.0f, 127.0f / 255.0f, 88.0f / 255.0f, 1.0f};
constexpr float LINE_THICKNESS = 2.0f;

}  // namespace

Lines::~Lines() = default;

void Lines::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

void Lines::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform lines.");
    }
    ImgTransformation source = *getTransformation();
    for(auto& line : lines) {
        line.startPoint = source.remapPointTo(target, line.startPoint);
        line.endPoint = source.remapPointTo(target, line.endPoint);
    }
    setTransformation(target);
}

Lines Lines::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Lines>::transformTo(target);
}

dai::VisualizeType Lines::getVisualizationMessage() const {
    ImgAnnotation annotation;

    // Each line is drawn as one two-point LINE_STRIP annotation.
    for(const auto& line : lines) {
        PointsAnnotation lineAnnotation;
        lineAnnotation.type = PointsAnnotationType::LINE_STRIP;
        lineAnnotation.points = {line.startPoint, line.endPoint};
        lineAnnotation.outlineColor = PRIMARY_COLOR;
        lineAnnotation.fillColor = PRIMARY_COLOR;
        lineAnnotation.thickness = LINE_THICKNESS;
        annotation.points.push_back(std::move(lineAnnotation));
    }

    auto annotations = std::make_shared<ImgAnnotations>();
    annotations->annotations.push_back(std::move(annotation));
    annotations->setTimestamp(getTimestamp());
    annotations->setSequenceNum(getSequenceNum());
    return annotations;
}

}  // namespace beta
}  // namespace dai
