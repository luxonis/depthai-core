#include "depthai/beta/datatype/Predictions.hpp"

#include <fmt/format.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

namespace dai {
namespace beta {

namespace {

// Visualization constants matching the source implementation in depthai-nodes.
constexpr double FONT_SIZE_PER_HEIGHT = 1.0 / 30.0;
const Color FONT_COLOR{1.0f, 1.0f, 1.0f, 1.0f};
const Color FONT_BACKGROUND_COLOR{0.0f, 0.0f, 0.0f, 0.0f};

}  // namespace

Predictions::~Predictions() = default;

void Predictions::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

float Predictions::getFirstPrediction() const {
    if(predictions.empty()) {
        throw std::runtime_error("Predictions message contains no predictions.");
    }
    return predictions.front().prediction;
}

void Predictions::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform predictions.");
    }
    setTransformation(target);
}

Predictions Predictions::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Predictions>::transformTo(target);
}

dai::VisualizeType Predictions::getVisualizationMessage() const {
    if(!transformation.has_value()) {
        return std::monostate{};
    }

    const auto [width, height] = transformation->getSize();
    if(width == 0 || height == 0) {
        return std::monostate{};
    }

    // Positions are computed in double precision like the Python source and only narrowed to
    // float when stored in the annotation.
    const double fontSize = FONT_SIZE_PER_HEIGHT * static_cast<double>(height);
    const double relativeFontSize = fontSize / static_cast<double>(height);
    const double xOffset = 3.0 / static_cast<double>(width);
    const double yOffset = 3.0 / static_cast<double>(height);

    ImgAnnotation annotation;
    for(std::size_t i = 0; i < predictions.size(); ++i) {
        const double yPosition = yOffset + relativeFontSize + static_cast<double>(i) * relativeFontSize;
        TextAnnotation text;
        text.position = Point2f(static_cast<float>(xOffset), static_cast<float>(yPosition), true);
        text.text = fmt::format("{:.2f}", predictions[i].prediction);
        text.fontSize = static_cast<float>(fontSize);
        text.textColor = FONT_COLOR;
        text.backgroundColor = FONT_BACKGROUND_COLOR;
        annotation.texts.push_back(std::move(text));
    }

    auto annotations = std::make_shared<ImgAnnotations>();
    annotations->annotations.push_back(std::move(annotation));
    annotations->setTimestamp(getTimestamp());
    annotations->setSequenceNum(getSequenceNum());
    return annotations;
}

}  // namespace beta
}  // namespace dai
