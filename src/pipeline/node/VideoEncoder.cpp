#include "depthai/pipeline/node/VideoEncoder.hpp"

#include <stdexcept>

#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

std::shared_ptr<VideoEncoder> VideoEncoder::build(Node::Output& input) {
    input.link(this->input);
    return std::static_pointer_cast<VideoEncoder>(shared_from_this());
}
void VideoEncoder::setNumFramesPool(int frames) {
    properties.numFramesPool = frames;
}

int VideoEncoder::getNumFramesPool() const {
    return properties.numFramesPool;
}

void VideoEncoder::setRateControlMode(VideoEncoderProperties::RateControlMode mode) {
    properties.rateCtrlMode = mode;
}

void VideoEncoder::setProfile(VideoEncoderProperties::Profile profile) {
    properties.profile = profile;
}

void VideoEncoder::setBitrate(int bitrate) {
    properties.bitrate = bitrate;
}

void VideoEncoder::setBitrateKbps(int bitrateKbps) {
    properties.bitrate = bitrateKbps * 1000;
}

void VideoEncoder::setKeyframeFrequency(int freq) {
    properties.keyframeFrequency = freq;
}

void VideoEncoder::setNumBFrames(int numBFrames) {
    properties.numBFrames = numBFrames;
}

void VideoEncoder::setQuality(int quality) {
    properties.quality = quality;
}

void VideoEncoder::setLossless(bool lossless) {
    properties.lossless = lossless;
}

void VideoEncoder::setFrameRate(float frameRate) {
    properties.frameRate = frameRate;
}

void VideoEncoder::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

VideoEncoderProperties::RateControlMode VideoEncoder::getRateControlMode() const {
    return properties.rateCtrlMode;
}

VideoEncoderProperties::Profile VideoEncoder::getProfile() const {
    return properties.profile;
}

int VideoEncoder::getBitrate() const {
    return properties.bitrate;
}

int VideoEncoder::getBitrateKbps() const {
    return properties.bitrate / 1000;
}

int VideoEncoder::getKeyframeFrequency() const {
    return properties.keyframeFrequency;
}

int VideoEncoder::getNumBFrames() const {
    return properties.numBFrames;
}

int VideoEncoder::getQuality() const {
    return properties.quality;
}

float VideoEncoder::getFrameRate() const {
    return properties.frameRate;
}

void VideoEncoder::setDefaultProfilePreset(float fps, VideoEncoderProperties::Profile profile) {
    setProfile(profile);
    setFrameRate(fps);

    switch(profile) {
        case VideoEncoderProperties::Profile::MJPEG:
            properties.quality = 95;
            break;

        case VideoEncoderProperties::Profile::H264_BASELINE:
        case VideoEncoderProperties::Profile::H264_HIGH:
        case VideoEncoderProperties::Profile::H264_MAIN:
        case VideoEncoderProperties::Profile::H265_MAIN:
            properties.keyframeFrequency = static_cast<int32_t>(fps);
            setBitrate(0);  // 0 = firmware computes the bitrate from size and fps
            break;

        default:
            break;
    }
}

bool VideoEncoder::getLossless() const {
    return properties.lossless;
}

int VideoEncoder::getMaxOutputFrameSize() const {
    return properties.outputFrameSize;
}

}  // namespace node
}  // namespace dai
