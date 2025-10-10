#include "depthai/pipeline/datatype/RGBDData.hpp"

namespace dai {

RGBDData::~RGBDData() = default;

void RGBDData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::RGBDData;
}

void RGBDData::setRGBFrame(const std::shared_ptr<ImgFrame>& frame) {
    frames["rgb"] = frame;
}

void RGBDData::setDepthFrame(const std::shared_ptr<ImgFrame>& frame) {
    frames["depth"] = frame;
}

std::shared_ptr<ImgFrame> RGBDData::getRGBFrame() {
    return std::dynamic_pointer_cast<ImgFrame>(frames["rgb"]);
}

std::shared_ptr<ImgFrame> RGBDData::getDepthFrame() {
    return std::dynamic_pointer_cast<ImgFrame>(frames["depth"]);
}
}  // namespace dai
