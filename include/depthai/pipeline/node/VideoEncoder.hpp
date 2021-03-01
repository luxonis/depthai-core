#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/VideoEncoderProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief VideoEncoder node. Encodes frames into MJPEG, H264 or H265.
 */
class VideoEncoder : public Node {
    using Properties = dai::VideoEncoderProperties;
    Properties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Input for NV12 ImgFrame to be encoded
     * Default queue is blocking with size set by 'setNumFramesPool' (4).
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 4, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries BITSTREAM encoded (MJPEG, H264 or H265) frame data.
     */
    Output bitstream{*this, "bitstream", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Sets default options for a specified size and profile
    /**
     * Sets a default preset based on specified input size, frame rate and profile
     * @param width Input frame width
     * @param height Input frame height
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    void setDefaultProfilePreset(int width, int height, float fps, Properties::Profile profile);

    /**
     * Sets a default preset based on specified input size, frame rate and profile
     * @param size Input frame size
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    void setDefaultProfilePreset(std::tuple<int, int> size, float fps, Properties::Profile profile);

    // node properties
    /**
     * Set number of frames in pool
     * @param frames Number of pool frames
     */
    void setNumFramesPool(int frames);

    /**
     * Get number of frames in pool
     * @returns Number of pool frames
     */
    int getNumFramesPool() const;

    // encoder properties
    /// Set rate control mode
    void setRateControlMode(Properties::RateControlMode mode);
    /// Set encoding profile
    void setProfile(int width, int height, Properties::Profile profile);
    /// Set output bitrate in kbps. Final bitrate depends on rate control mode
    void setBitrate(int bitrateKbps);

    /**
     * Set keyframe frequency. Every Nth frame a keyframe is inserted.
     *
     * Applicable only to H264 and H265 profiles
     *
     * Examples:
     *
     *  - 30 FPS video, keyframe frequency: 30. Every 1s a keyframe will be inserted
     *
     *  - 60 FPS video, keyframe frequency: 180. Every 3s a keyframe will be inserted
     *
     */
    void setKeyframeFrequency(int freq);

    /// Set number of B frames to be inserted
    void setNumBFrames(int numBFrames);

    /**
     * Set quality
     * @param quality Value between 0-100%. Approximates quality
     */
    void setQuality(int quality);

    /**
     * Sets expected frame rate
     * @param frameRate Frame rate in frames per second
     */
    void setFrameRate(int frameRate);

    /// Get rate control mode
    Properties::RateControlMode getRateControlMode() const;
    /// Get profile
    Properties::Profile getProfile() const;
    /// Get bitrate
    int getBitrate() const;
    /// Get keyframe frequency
    int getKeyframeFrequency() const;
    // int getMaxBitrate() const;
    /// Get number of B frames
    int getNumBFrames() const;
    /// Get quality
    int getQuality() const;
    /// Get input size
    std::tuple<int, int> getSize() const;
    /// Get input width
    int getWidth() const;
    /// Get input height
    int getHeight() const;
    /// Get frame rate
    int getFrameRate() const;
};

}  // namespace node
}  // namespace dai
