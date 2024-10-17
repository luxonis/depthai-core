#include "depthai/pipeline/datatype/EncodedFrame.hpp"

#include "utility/H26xParsers.hpp"

namespace dai {
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

std::unique_ptr<google::protobuf::Message> EncodedFrame::getProtoMessage() const {
    // Create a unique pointer to the protobuf EncodedFrame message
    auto encodedFrame = std::make_unique<proto::encoded_frame::EncodedFrame>();

    // Populate the protobuf message fields with the EncodedFrame data
    encodedFrame->set_instancenum(this->instanceNum);  // instanceNum -> instancenum
    encodedFrame->set_width(this->width);
    encodedFrame->set_height(this->height);
    encodedFrame->set_quality(this->quality);
    encodedFrame->set_bitrate(this->bitrate);
    encodedFrame->set_profile(static_cast<proto::encoded_frame::Profile>(this->profile));  // Profile enum
    encodedFrame->set_lossless(this->lossless);
    encodedFrame->set_type(static_cast<proto::encoded_frame::FrameType>(this->type));  // FrameType enum
    encodedFrame->set_frameoffset(this->frameOffset);                                  // frameOffset -> frameoffset
    encodedFrame->set_framesize(this->frameSize);                                      // frameSize -> framesize
    encodedFrame->set_sequencenum(this->sequenceNum);                                  // sequenceNum -> sequencenum

    // Set timestamps
    proto::common::Timestamp* ts = encodedFrame->mutable_ts();
    ts->set_sec(this->ts.sec);
    ts->set_nsec(this->ts.nsec);

    proto::common::Timestamp* tsDevice = encodedFrame->mutable_tsdevice();
    tsDevice->set_sec(this->tsDevice.sec);
    tsDevice->set_nsec(this->tsDevice.nsec);

    // Set camera settings
    proto::common::CameraSettings* cam = encodedFrame->mutable_cam();
    cam->set_exposuretimeus(this->cam.exposureTimeUs);    // exposureTimeUs -> exposuretimeus
    cam->set_sensitivityiso(this->cam.sensitivityIso);    // sensitivityIso -> sensitivityiso
    cam->set_lensposition(this->cam.lensPosition);        // lensPosition -> lensposition
    cam->set_wbcolortemp(this->cam.wbColorTemp);          // wbColorTemp -> wbcolortemp
    cam->set_lenspositionraw(this->cam.lensPositionRaw);  // lensPositionRaw -> lenspositionraw

    // Set the encoded frame data
    encodedFrame->set_data(this->data->getData().data(), this->data->getData().size());

    // Return the populated protobuf message
    return encodedFrame;
}

}  // namespace dai
