#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/VideoEncoderProperties.hpp>

namespace dai {
namespace node {
class VideoEncoder : public Node {
    dai::VideoEncoderProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, true}}};
    Output bitstream{*this, "bitstream", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Sets default options for a specified size and profile
    void setDefaultProfilePreset(int width, int height, float fps, VideoEncoderProperties::Profile profile);
    void setDefaultProfilePreset(std::tuple<int, int> size, float fps, VideoEncoderProperties::Profile profile);

    // node properties
    void setNumFramesPool(int frames);
    int getNumFramesPool() const;

    // encoder properties
    void setRateControlMode(VideoEncoderProperties::RateControlMode mode);
    void setProfile(VideoEncoderProperties::Profile profile);
    void setBitrate(int bitrateKbps);
    void setKeyframeFrequency(int freq);
    // void setMaxBitrate(int maxBitrateKbps);
    void setNumBFrames(int numBFrames);
    void setQuality(int quality);
    void setWidth(int width);
    void setHeight(int height);
    void setFrameRate(int frameRate);

    VideoEncoderProperties::RateControlMode getRateControlMode() const;
    VideoEncoderProperties::Profile getProfile() const;
    int getBitrate() const;
    int getKeyframeFrequency() const;
    // int getMaxBitrate() const;
    int getNumBFrames() const;
    int getQuality() const;
    int getWidth() const;
    int getHeight() const;
    int getFrameRate() const;
};

}  // namespace node
}  // namespace dai
