#include "depthai/beta/datatype/Classifications.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

namespace dai {
namespace beta {

namespace {

// Visualization constants matching the source implementation in depthai-nodes.
constexpr float FONT_SIZE_PER_HEIGHT = 1.0f / 30.0f;
constexpr std::size_t MAX_VISUALIZED_CLASSES = 5;
const Color FONT_COLOR{1.0f, 1.0f, 1.0f, 1.0f};
const Color FONT_BACKGROUND_COLOR{0.0f, 0.0f, 0.0f, 0.0f};

}  // namespace

Classifications::~Classifications() = default;

void Classifications::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

std::string Classifications::getTopClass() const {
    if(classes.empty()) {
        throw std::runtime_error("Classifications message contains no classes.");
    }
    return classes.front();
}

float Classifications::getTopScore() const {
    if(scores.empty()) {
        throw std::runtime_error("Classifications message contains no scores.");
    }
    return scores.front();
}

void Classifications::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform classifications.");
    }
    setTransformation(target);
}

Classifications Classifications::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Classifications>::transformTo(target);
}

dai::VisualizeType Classifications::getVisualizationMessage() const {
    if(!transformation.has_value()) {
        return std::monostate{};
    }

    const auto [width, height] = transformation->getSize();
    if(width == 0 || height == 0) {
        return std::monostate{};
    }

    const float fontSize = FONT_SIZE_PER_HEIGHT * static_cast<float>(height);
    const float relativeFontSize = fontSize / static_cast<float>(height);
    const float xOffset = 2.0f / static_cast<float>(width);
    const float yOffset = 2.0f / static_cast<float>(height);

    ImgAnnotation annotation;
    const std::size_t numClasses = std::min({MAX_VISUALIZED_CLASSES, classes.size(), scores.size()});
    for(std::size_t i = 0; i < numClasses; ++i) {
        const float yPosition = yOffset + relativeFontSize + static_cast<float>(i) * relativeFontSize;
        TextAnnotation text;
        text.position = Point2f(xOffset, yPosition, true);
        text.text = fmt::format("{} {:.0f}%", classes[i], scores[i] * 100.0f);
        text.fontSize = fontSize;
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
