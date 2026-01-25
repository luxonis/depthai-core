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
    std::visit([&](auto&& arg) { this->colorFrame = arg; }, frame);
}

void RGBDData::setDepthFrame(const FrameVariant& frame) {
    // Unpack the variant and assign the contained pointer to depthFrame
    std::visit([&](auto&& arg) { this->depthFrame = arg; }, frame);
}

// Getters
std::optional<RGBDData::FrameVariant> RGBDData::getRGBFrame() const {
    if(!colorFrame) {
        // color frame is not set and we don't know what type it is, so we return std::nullopt
        // (we don't want to commit to a specific pointer type)
        return std::nullopt;
    }

    auto type = colorFrame->getDatatype();
    if(type == DatatypeEnum::ImgFrame) {
        return std::dynamic_pointer_cast<ImgFrame>(colorFrame);
    }
    if(type == DatatypeEnum::EncodedFrame) {
        return std::dynamic_pointer_cast<EncodedFrame>(colorFrame);
    }
    throw std::runtime_error("Unhandled frame type in RGBDData color frame variant");
}

std::optional<RGBDData::FrameVariant> RGBDData::getDepthFrame() const {
    if(!depthFrame) {
        // depth frame is not set and we don't know what type it is, so we return std::nullopt
        // (we don't want to commit to a specific pointer type)
        return std::nullopt;
    }

    auto type = depthFrame->getDatatype();
    if(type == DatatypeEnum::ImgFrame) {
        return std::dynamic_pointer_cast<ImgFrame>(depthFrame);
    }
    if(type == DatatypeEnum::EncodedFrame) {
        return std::dynamic_pointer_cast<EncodedFrame>(depthFrame);
    }
    throw std::runtime_error("Unhandled frame type in RGBDData depth frame variant");
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
