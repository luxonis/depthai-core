#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

#include "utility/H26xParsers.hpp"

namespace dai {

EncodedFrame::~EncodedFrame() = default;

void EncodedFrame::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::EncodedFrame;
}

// getters
unsigned int EncodedFrame::getInstanceNum() const {
    return instanceNum;
}
unsigned int EncodedFrame::getHeight() const {
    return height;
}
unsigned int EncodedFrame::getWidth() const {
    return width;
}
std::chrono::microseconds EncodedFrame::getExposureTime() const {
    return std::chrono::microseconds(cam.exposureTimeUs);
}
int EncodedFrame::getSensitivity() const {
    return cam.sensitivityIso;
}
int EncodedFrame::getColorTemperature() const {
    return cam.wbColorTemp;
}
int EncodedFrame::getLensPosition() const {
    return cam.lensPosition;
}
float EncodedFrame::getLensPositionRaw() const {
    return cam.lensPositionRaw;
}
unsigned int EncodedFrame::getQuality() const {
    return quality;
}
unsigned int EncodedFrame::getBitrate() const {
    return bitrate;
}
bool EncodedFrame::getLossless() const {
    return lossless;
}
EncodedFrame::FrameType EncodedFrame::getFrameType() {
    if(type == FrameType::Unknown) {
        utility::SliceType frameType = utility::SliceType::Unknown;
        switch(profile) {
            case EncodedFrame::Profile::JPEG:
                frameType = utility::SliceType::I;
                break;
            case EncodedFrame::Profile::AVC:
                // TODO(Morato) - change this to zero copy
                frameType = utility::getTypesH264(std::vector<uint8_t>(data->getData().begin(), data->getData().end()), true)[0];
                break;
            case EncodedFrame::Profile::HEVC:
                frameType = utility::getTypesH265(std::vector<uint8_t>(data->getData().begin(), data->getData().end()), true)[0];
                break;
        }
        switch(frameType) {
            case utility::SliceType::P:
                type = FrameType::P;
                break;
            case utility::SliceType::B:
                type = FrameType::B;
                break;
            case utility::SliceType::I:
                type = FrameType::I;
                break;
            case utility::SliceType::SP:
                type = FrameType::P;
                break;
            case utility::SliceType::SI:
                type = FrameType::I;
                break;
            case utility::SliceType::Unknown:
                type = FrameType::Unknown;
                break;
        }
    }
    return type;
}
EncodedFrame::Profile EncodedFrame::getProfile() const {
    return profile;
}

EncodedFrame& EncodedFrame::setInstanceNum(unsigned int instanceNum) {
    this->instanceNum = instanceNum;
    return *this;
}
EncodedFrame& EncodedFrame::setWidth(unsigned int width) {
    this->width = width;
    return *this;
}
EncodedFrame& EncodedFrame::setHeight(unsigned int height) {
    this->height = height;
    return *this;
}
EncodedFrame& EncodedFrame::setSize(unsigned int width, unsigned int height) {
    setWidth(width);
    setHeight(height);
    return *this;
}
EncodedFrame& EncodedFrame::setSize(std::tuple<unsigned int, unsigned int> size) {
    setSize(std::get<0>(size), std::get<1>(size));
    return *this;
}
EncodedFrame& EncodedFrame::setQuality(unsigned int quality) {
    this->quality = quality;
    return *this;
}
EncodedFrame& EncodedFrame::setBitrate(unsigned int bitrate) {
    this->bitrate = bitrate;
    return *this;
}

EncodedFrame& EncodedFrame::setLossless(bool lossless) {
    this->lossless = lossless;
    return *this;
}
EncodedFrame& EncodedFrame::setFrameType(FrameType frameType) {
    this->type = frameType;
    return *this;
}
EncodedFrame& EncodedFrame::setProfile(Profile profile) {
    this->profile = profile;
    return *this;
}

ImgFrame EncodedFrame::getImgFrameMeta() const {
    ImgFrame frame;
    frame.cam = cam;
    frame.setInstanceNum(instanceNum);
    frame.setWidth(width);
    frame.setHeight(height);
    frame.setType(ImgFrame::Type::BITSTREAM);
    frame.setSourceSize(transformation.getSourceSize());
    // Important to set the transformation last as setting the source size resets it
    frame.transformation = transformation;
    frame.setSequenceNum(getSequenceNum());
    frame.setTimestamp(getTimestamp());
    frame.setTimestampDevice(getTimestampDevice());
    frame.setTimestampSystem(getTimestampSystem());
    return frame;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair EncodedFrame::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> EncodedFrame::serializeProto(bool metadataOnly) const {
    return utility::serializeProto(utility::getProtoMessage(this, metadataOnly));
}
#endif

}  // namespace dai
