#include "depthai/pipeline/datatype/RGBDData.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/RGBDData.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

#include <memory>
namespace dai {

RGBDData::~RGBDData() = default;

void RGBDData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::RGBDData;
}

// Setters
void RGBDData::setRGBFrame(const FrameVariant& frame) {
    // Unpack the variant and assign the contained pointer to colorFrame
    std::visit([&](auto&& arg) { colorFrame = arg; }, frame);
}

void RGBDData::setDepthFrame(const FrameVariant& frame) {
    std::visit([&](auto&& arg) { depthFrame = arg; }, frame);
}

// Getters
std::optional<RGBDData::FrameVariant> RGBDData::getRGBFrame() const {
    if(!colorFrame.has_value()) {
        return std::nullopt;
    }

    auto type = colorFrame.value()->getDatatype();
    if(type == DatatypeEnum::ImgFrame) {
        return std::make_optional(std::dynamic_pointer_cast<ImgFrame>(colorFrame.value()));
    } else if(type == DatatypeEnum::EncodedFrame) {
        return std::make_optional(std::dynamic_pointer_cast<EncodedFrame>(colorFrame.value()));
    } else {
        throw std::runtime_error("Unhandled frame type in RGBDData color frame variant");
    }
}

std::optional<RGBDData::FrameVariant> RGBDData::getDepthFrame() const {
    if(!depthFrame.has_value()) {
        return std::nullopt;
    }

    auto type = depthFrame->get()->getDatatype();
    if(type == DatatypeEnum::ImgFrame) {
        return std::make_optional(std::dynamic_pointer_cast<ImgFrame>(depthFrame.value()));
    } else if(type == DatatypeEnum::EncodedFrame) {
        return std::make_optional(std::dynamic_pointer_cast<EncodedFrame>(depthFrame.value()));
    } else {
        throw std::runtime_error("Unhandled frame type in RGBDData depth frame variant");
    }
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
std::vector<std::uint8_t> RGBDData::serializeProto(bool metadataOnly) const {
    return utility::serializeProto(utility::getProtoMessage(this, metadataOnly));
}

ProtoSerializable::SchemaPair RGBDData::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
