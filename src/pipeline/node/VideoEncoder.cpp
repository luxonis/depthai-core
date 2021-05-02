#include "depthai/pipeline/node/VideoEncoder.hpp"

namespace dai {
namespace node {

VideoEncoder::VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string VideoEncoder::getName() const {
    return "VideoEncoder";
}

std::vector<Node::Input> VideoEncoder::getInputs() {
    return {input};
}

std::vector<Node::Output> VideoEncoder::getOutputs() {
    return {bitstream};
}

nlohmann::json VideoEncoder::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> VideoEncoder::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

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

void VideoEncoder::setProfile(std::tuple<int, int> size, VideoEncoderProperties::Profile profile) {
    setProfile(std::get<0>(size), std::get<1>(size), profile);
}

void VideoEncoder::setProfile(int width, int height, VideoEncoderProperties::Profile profile) {
    // Width & height H26x limitations
    if(profile != VideoEncoderProperties::Profile::MJPEG) {
        if(width % 8 != 0 || height % 8 != 0) {
            throw std::invalid_argument("VideoEncoder - Width and height must be multiple of 8 for H26x encoder profile");
        }
        if(width > 4096 || height > 4096) {
            throw std::invalid_argument("VideoEncoder - Width and height must be smaller than 4096 for H26x encoder profile");
        }
    } else {
        // width & height MJPEG limitations
        if(width % 16 != 0 || height % 2 != 0) {
            throw std::invalid_argument("VideoEncoder - Width must be multiple of 16 and height multiple of 2 for MJPEG encoder profile");
        }
        if(width > 16384 || height > 8192) {
            throw std::invalid_argument("VideoEncoder - Width must be smaller or to 16384 and height to 8192");
        }
    }

    properties.width = width;
    properties.height = height;
    properties.profile = profile;
}

void VideoEncoder::setBitrate(int bitrate) {
    properties.bitrate = bitrate;
    properties.maxBitrate = bitrate;
}

void VideoEncoder::setBitrateKbps(int bitrateKbps) {
    properties.bitrate = bitrateKbps * 1000;
    properties.maxBitrate = bitrateKbps * 1000;
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
    return {properties.width, properties.height};
}

int VideoEncoder::getWidth() const {
    return std::get<0>(getSize());
}

int VideoEncoder::getHeight() const {
    return std::get<1>(getSize());
}

float VideoEncoder::getFrameRate() const {
    return properties.frameRate;
}

void VideoEncoder::setDefaultProfilePreset(int width, int height, float fps, VideoEncoderProperties::Profile profile) {
    // Checks

    // Set properties
    setProfile(width, height, profile);
    setFrameRate(fps);

    switch(profile) {
        case VideoEncoderProperties::Profile::MJPEG:
            properties.quality = 95;
            break;

        case VideoEncoderProperties::Profile::H264_BASELINE:
        case VideoEncoderProperties::Profile::H264_HIGH:
        case VideoEncoderProperties::Profile::H264_MAIN:
        case VideoEncoderProperties::Profile::H265_MAIN: {
            // By default set keyframe frequency to equal fps
            properties.keyframeFrequency = static_cast<int32_t>(fps);

            // Approximate bitrate on input w/h and fps
            constexpr float ESTIMATION_FPS = 30.0f;
            constexpr float AREA_MUL = 1.1f;

            // calculate pixel area
            const int pixelArea = width * height;
            if(pixelArea <= 1280 * 720 * AREA_MUL) {
                // 720p
                setBitrateKbps(static_cast<int>((4000 / ESTIMATION_FPS) * fps));
            } else if(pixelArea <= 1920 * 1080 * AREA_MUL) {
                // 1080p
                setBitrateKbps(static_cast<int>((8500 / ESTIMATION_FPS) * fps));
            } else if(pixelArea <= 2560 * 1440 * AREA_MUL) {
                // 1440p
                setBitrateKbps(static_cast<int>((14000 / ESTIMATION_FPS) * fps));
            } else {
                // 4K
                setBitrateKbps(static_cast<int>((20000 / ESTIMATION_FPS) * fps));
            }
        } break;

        default:
            break;
    }
}

void VideoEncoder::setDefaultProfilePreset(std::tuple<int, int> size, float fps, VideoEncoderProperties::Profile profile) {
    setDefaultProfilePreset(std::get<0>(size), std::get<1>(size), fps, profile);
}

bool VideoEncoder::getLossless() const {
    return properties.lossless;
}

}  // namespace node
}  // namespace dai
