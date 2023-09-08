#include "depthai/pipeline/datatype/EncodedFrame.hpp"

namespace dai {

std::shared_ptr<RawBuffer> EncodedFrame::serialize() const {
    return raw;
}

EncodedFrame::EncodedFrame() : Buffer(std::make_shared<RawEncodedFrame>()), frame(*dynamic_cast<RawEncodedFrame*>(raw.get())) {
    // set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}
EncodedFrame::EncodedFrame(std::shared_ptr<RawEncodedFrame> ptr) : Buffer(std::move(ptr)), frame(*dynamic_cast<RawEncodedFrame*>(raw.get())) {}

// Getters

// getters
unsigned int EncodedFrame::getWidth() const {
    return frame.width;
}
unsigned int EncodedFrame::getHeight() const {
    return frame.height;
}
unsigned int EncodedFrame::getQuality() const {
    return frame.quality;
}
bool EncodedFrame::getLossless() const {
    return frame.lossless;
}
EncodedFrame::FrameType EncodedFrame::getFrameType() const {
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
EncodedFrame& EncodedFrame::setWidth(unsigned int width) {
    frame.width = width;
    return *this;
}
EncodedFrame& EncodedFrame::setHeight(unsigned int height) {
    frame.height = height;
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
    frame.quality = quality;
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

}