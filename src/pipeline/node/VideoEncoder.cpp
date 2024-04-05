#include "depthai/pipeline/node/VideoEncoder.hpp"

// std
#include <stdexcept>

// libraries
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

// node properties
void VideoEncoder::setNumFramesPool(int frames) {
    properties.numFramesPool = frames;
    // Set default input queue size as well
    input.defaultQueueSize = frames;
}

int VideoEncoder::getNumFramesPool() const {
    return properties.numFramesPool;
}

// encoder properties
void VideoEncoder::setRateControlMode(VideoEncoderProperties::RateControlMode mode) {
    properties.rateCtrlMode = mode;
}

void VideoEncoder::setProfile(VideoEncoderProperties::Profile profile) {
    properties.profile = profile;
}

void VideoEncoder::setProfile(std::tuple<int, int> size, VideoEncoderProperties::Profile profile) {
    (void)size;
    logger::warn("VideoEncoder {}: passing 'size' is deprecated. It is auto-determined from first frame", __func__);
    setProfile(profile);
}

void VideoEncoder::setProfile(int width, int height, VideoEncoderProperties::Profile profile) {
    (void)width;
    (void)height;
    logger::warn("VideoEncoder {}: passing 'width'/ 'height' is deprecated. The size is auto-determined from first frame", __func__);
    setProfile(profile);
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

// int VideoEncoder::getMaxBitrate() const {
//    return properties.maxBitrate;
//}

int VideoEncoder::getNumBFrames() const {
    return properties.numBFrames;
}

int VideoEncoder::getQuality() const {
    return properties.quality;
}

std::tuple<int, int> VideoEncoder::getSize() const {
    logger::warn("VideoEncoder {} is deprecated. The size is auto-determined from first frame and not known upfront", __func__);
    return {0, 0};
}

int VideoEncoder::getWidth() const {
    logger::warn("VideoEncoder {} is deprecated. The size is auto-determined from first frame and not known upfront", __func__);
    return 0;
}

int VideoEncoder::getHeight() const {
    logger::warn("VideoEncoder {} is deprecated. The size is auto-determined from first frame and not known upfront", __func__);
    return 0;
}

float VideoEncoder::getFrameRate() const {
    return properties.frameRate;
}

void VideoEncoder::setDefaultProfilePreset(float fps, VideoEncoderProperties::Profile profile) {
    // Checks

    // Set properties
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
            // By default set keyframe frequency to equal fps
            properties.keyframeFrequency = static_cast<int32_t>(fps);
            // A default bitrate computed by firmware based on size and fps
            setBitrate(0);
            break;

        default:
            break;
    }
}

void VideoEncoder::setDefaultProfilePreset(int width, int height, float fps, VideoEncoderProperties::Profile profile) {
    (void)width;
    (void)height;
    logger::warn("VideoEncoder {}: passing 'width'/ 'height' is deprecated. The size is auto-determined from first frame", __func__);
    setDefaultProfilePreset(fps, profile);
}

void VideoEncoder::setDefaultProfilePreset(std::tuple<int, int> size, float fps, VideoEncoderProperties::Profile profile) {
    (void)size;
    logger::warn("VideoEncoder {}: passing 'width'/ 'height' is deprecated. The size is auto-determined from first frame", __func__);
    setDefaultProfilePreset(fps, profile);
}

bool VideoEncoder::getLossless() const {
    return properties.lossless;
}

int VideoEncoder::getMaxOutputFrameSize() const {
    return properties.outputFrameSize;
}

void VideoEncoder::setMisc(std::string property, std::string value) {
    properties.miscProps.push_back(std::make_pair(property, value));
}

void VideoEncoder::setMisc(std::string property, int value) {
    setMisc(property, std::to_string(value));
}

void VideoEncoder::setMisc(std::string property, float value) {
    setMisc(property, std::to_string(value));
}

void VideoEncoder::clearMiscProperties() {
    properties.miscProps.clear();
}

std::vector<std::pair<std::string, std::string>> VideoEncoder::getMiscProperties() {
    return properties.miscProps;
}

}  // namespace node
}  // namespace dai
