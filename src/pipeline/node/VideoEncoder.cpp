#include "depthai/pipeline/node/VideoEncoder.hpp"

namespace dai {
namespace node {

    VideoEncoder::VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

    std::string VideoEncoder::getName() {
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

    void VideoEncoder::setDefaultProfilePreset(int width, int height, float fps, VideoEncoderProperties::Profile profile) {
        properties.width = width;
        properties.height = height;
        properties.frameRate = fps;
        properties.profile = profile;

        switch(profile) {
            case VideoEncoderProperties::Profile::MJPEG:
                properties.quality = 95;
                break;

            case VideoEncoderProperties::Profile::H264_BASELINE:
            case VideoEncoderProperties::Profile::H264_HIGH:
            case VideoEncoderProperties::Profile::H264_MAIN:
            case VideoEncoderProperties::Profile::H265_MAIN: {
                // By default set keyframe frequency to equal fps
                properties.keyframeFrequency = fps;

                // Approximate bitrate on input w/h and fps
                constexpr float ESTIMATION_FPS = 30.0f;
                constexpr float AREA_MUL = 1.1f;

                // calculate pixel area
                const int pixelArea = width * height;
                if(pixelArea <= 1280 * 720 * AREA_MUL) {
                    // 720p
                    setBitrate((4000*1000 / ESTIMATION_FPS) * fps);
                } else if(pixelArea <= 1920 * 1080 * AREA_MUL) {
                    // 1080p
                    setBitrate((8500*1000 / ESTIMATION_FPS) * fps);
                } else if(pixelArea <= 2560 * 1440 * AREA_MUL) {
                    // 1440p
                    setBitrate((14000*1000 / ESTIMATION_FPS) * fps);
                } else {
                    // 4K
                    setBitrate((20000*1000 / ESTIMATION_FPS) * fps);
                }
            } break;

            default:
                break;
        }
    }

    // node properties
    void VideoEncoder::setNumFramesPool(int frames) {
        properties.numFramesPool = frames;
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

    void VideoEncoder::setBitrate(int bitrate) {
        properties.bitrate = bitrate;
        properties.maxBitrate = bitrate;
    }

    void VideoEncoder::setKeyframeFrequency(int freq) {
        properties.keyframeFrequency = freq;
    }

    // Max bitrate and bitrate must match
    //void VideoEncoder::setMaxBitrate(int maxBitrateKbps) {
    //    properties.maxBitrate = maxBitrateKbps;
    //}

    void VideoEncoder::setNumBFrames(int numBFrames) {
        properties.numBFrames = numBFrames;
    }

    void VideoEncoder::setQuality(int quality) {
        properties.quality = quality;
    }

    void VideoEncoder::setWidth(int width) {
        properties.width = width;
    }

    void VideoEncoder::setHeight(int height) {
        properties.height = height;
    }

    void VideoEncoder::setFrameRate(int frameRate) {
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

    int VideoEncoder::getKeyframeFrequency() const {
        return properties.keyframeFrequency;
    }

    //int VideoEncoder::getMaxBitrate() const {
    //    return properties.maxBitrate;
    //}

    int VideoEncoder::getNumBFrames() const {
        return properties.numBFrames;
    }

    int VideoEncoder::getQuality() const {
        return properties.quality;
    }

    int VideoEncoder::getWidth() const {
        return properties.width;
    }

    int VideoEncoder::getHeight() const {
        return properties.height;
    }

    int VideoEncoder::getFrameRate() const {
        return properties.frameRate;
    }

}  // namespace node
}  // namespace dai
