#include "depthai/pipeline/datatype/AprilTags.hpp"

namespace dai {

AprilTags::~AprilTags() = default;

void AprilTags::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::AprilTags;
};

AprilTags AprilTags::transformTo(const ImgTransformation& target) {
    return TransformableCRTP<AprilTags>::transformTo(target);
}

void AprilTags::transformToInternal(const ImgTransformation& target) {
    if(!this->transformation.has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform AprilTags.");
    }
    ImgTransformation source = *this->transformation;

    for(auto& aprilTag : aprilTags) {
        aprilTag.topLeft = source.remapPointTo(target, aprilTag.topLeft);
        aprilTag.topRight = source.remapPointTo(target, aprilTag.topRight);
        aprilTag.bottomRight = source.remapPointTo(target, aprilTag.bottomRight);
        aprilTag.bottomLeft = source.remapPointTo(target, aprilTag.bottomLeft);
    }
    setTransformation(target);
}

}  // namespace dai
