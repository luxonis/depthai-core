#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"

namespace dai {

SegmentationParserConfig::~SegmentationParserConfig() = default;

void SegmentationParserConfig::setLabels(const std::vector<std::string>& labels_) {
    labels = labels_;
}

std::vector<std::string> SegmentationParserConfig::getLabels() const {
    return labels;
}

void SegmentationParserConfig::setConfidenceThreshold(float threshold) {
    if(threshold < 0.0f || threshold > 1.0f) {
        throw std::invalid_argument("Confidence threshold must be between 0.0 and 1.0");
    }
    confidenceThreshold = threshold;
}

float SegmentationParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void SegmentationParserConfig::setStepSize(int stepSize) {
    if(stepSize <= 0) {
        throw std::invalid_argument("Step size must be a positive integer");
    }
    this->stepSize = stepSize;
}

int SegmentationParserConfig::getStepSize() const {
    return stepSize;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
void SegmentationParserConfig::setOutputSize(size_t width, size_t height) {
    outputWidth = width;
    outputHeight = height;
}

void SegmentationParserConfig::setResizeMode(ResizeMode mode) {
    resizeMode = mode;
}
#endif

}  // namespace dai
