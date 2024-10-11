#include "depthai/pipeline/datatype/EncodedFrame.hpp"

#include "utility/H26xParsers.hpp"

namespace dai {

std::shared_ptr<RawBuffer> EncodedFrame::serialize() const {
    return raw;
}

EncodedFrame::EncodedFrame() : Buffer(std::make_shared<RawEncodedFrame>()), frame(*dynamic_cast<RawEncodedFrame*>(raw.get())) {
    // set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}
EncodedFrame::EncodedFrame(std::shared_ptr<RawEncodedFrame> ptr) : Buffer(std::move(ptr)), frame(*dynamic_cast<RawEncodedFrame*>(raw.get())) {}

// getters
unsigned int EncodedFrame::getInstanceNum() const {
    return frame.instanceNum;
}
std::chrono::microseconds EncodedFrame::getExposureTime() const {
    return std::chrono::microseconds(frame.cam.exposureTimeUs);
}
int EncodedFrame::getSensitivity() const {
    return frame.cam.sensitivityIso;
}
int EncodedFrame::getColorTemperature() const {
    return frame.cam.wbColorTemp;
}
int EncodedFrame::getLensPosition() const {
    return frame.cam.lensPosition;
}
float EncodedFrame::getLensPositionRaw() const {
    return frame.cam.lensPositionRaw;
}
unsigned int EncodedFrame::getQuality() const {
    return frame.quality;
}
unsigned int EncodedFrame::getBitrate() const {
    return frame.bitrate;
}
bool EncodedFrame::getLossless() const {
    return frame.lossless;
}
EncodedFrame::FrameType EncodedFrame::getFrameType() const {
    if(frame.type == FrameType::Unknown) {
        utility::SliceType frameType = utility::SliceType::Unknown;
        switch(frame.profile) {
            case RawEncodedFrame::Profile::JPEG:
                frameType = utility::SliceType::I;
                break;
            case RawEncodedFrame::Profile::AVC:
                frameType = utility::getTypesH264(frame.data, true)[0];
                break;
            case RawEncodedFrame::Profile::HEVC:
                frameType = utility::getTypesH265(frame.data, true)[0];
                break;
        }
        switch(frameType) {
            case utility::SliceType::P:
                frame.type = FrameType::P;
                break;
            case utility::SliceType::B:
                frame.type = FrameType::B;
                break;
            case utility::SliceType::I:
                frame.type = FrameType::I;
                break;
            case utility::SliceType::SP:
                frame.type = FrameType::P;
                break;
            case utility::SliceType::SI:
                frame.type = FrameType::I;
                break;
            case utility::SliceType::Unknown:
                frame.type = FrameType::Unknown;
                break;
        }
    }
    return frame.type;
}
EncodedFrame::Profile EncodedFrame::getProfile() const {
    return frame.profile;
}

// setters
EncodedFrame& EncodedFrame::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<EncodedFrame&>(Buffer::setTimestamp(tp));
}
EncodedFrame& EncodedFrame::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<EncodedFrame&>(Buffer::setTimestampDevice(tp));
}
EncodedFrame& EncodedFrame::setSequenceNum(int64_t sequenceNum) {
    return static_cast<EncodedFrame&>(Buffer::setSequenceNum(sequenceNum));
}
EncodedFrame& EncodedFrame::setInstanceNum(unsigned int instanceNum) {
    frame.instanceNum = instanceNum;
    return *this;
}
EncodedFrame& EncodedFrame::setQuality(unsigned int quality) {
    frame.quality = quality;
    return *this;
}
EncodedFrame& EncodedFrame::setBitrate(unsigned int bitrate) {
    frame.bitrate = bitrate;
    return *this;
}

EncodedFrame& EncodedFrame::setLossless(bool lossless) {
    frame.lossless = lossless;
    return *this;
}
EncodedFrame& EncodedFrame::setFrameType(FrameType frameType) {
    frame.type = frameType;
    return *this;
}
EncodedFrame& EncodedFrame::setProfile(Profile profile) {
    frame.profile = profile;
    return *this;
}

}  // namespace dai
