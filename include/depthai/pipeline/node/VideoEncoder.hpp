#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/VideoEncoderProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief VideoEncoder node. Encodes frames into MJPEG, H264 or H265.
 */
class VideoEncoder : public DeviceNodeCRTP<DeviceNode, VideoEncoder, VideoEncoderProperties> {
   public:
    constexpr static const char* NAME = "VideoEncoder";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    /**
     * Build the node by linking an input output.
     */
    std::shared_ptr<VideoEncoder> build(Node::Output& input);

    /**
     * Input for NV12 ImgFrame to be encoded
     */
    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message that carries BITSTREAM encoded (MJPEG, H264 or H265) frame data.
     * Mutually exclusive with out.
     */
    Output bitstream{*this, {"bitstream", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs EncodedFrame message that carries encoded (MJPEG, H264 or H265) frame data.
     * Mutually exclusive with bitstream.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::EncodedFrame, false}}}}};
    // Sets default options for a specified size and profile
    /**
     * Sets a default preset based on specified frame rate and profile
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    void setDefaultProfilePreset(float fps, Properties::Profile profile);

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
    void setProfile(Properties::Profile profile);

    /// Set output bitrate in bps, for CBR rate control mode. 0 for auto (based on frame size and FPS)
    void setBitrate(int bitrate);
    /// Set output bitrate in kbps, for CBR rate control mode. 0 for auto (based on frame size and FPS)
    void setBitrateKbps(int bitrateKbps);

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
     * Set lossless mode. Applies only to [M]JPEG profile
     * @param lossless True to enable lossless jpeg encoding, false otherwise
     */
    void setLossless(bool lossless);

    /**
     * Sets expected frame rate
     * @param frameRate Frame rate in frames per second
     */
    void setFrameRate(float frameRate);

    /**
     * Specifies maximum output encoded frame size
     */
    void setMaxOutputFrameSize(int maxFrameSize);

    /// Get rate control mode
    Properties::RateControlMode getRateControlMode() const;
    /// Get profile
    Properties::Profile getProfile() const;
    /// Get bitrate in bps
    int getBitrate() const;
    /// Get bitrate in kbps
    int getBitrateKbps() const;
    /// Get keyframe frequency
    int getKeyframeFrequency() const;
    // int getMaxBitrate() const;
    /// Get number of B frames
    int getNumBFrames() const;
    /// Get quality
    int getQuality() const;

    /// Get frame rate
    float getFrameRate() const;
    /// Get lossless mode. Applies only when using [M]JPEG profile.
    bool getLossless() const;
    /**
     * Get the maximum encoded frame size in bytes.
     */
    int getMaxOutputFrameSize() const;
};

}  // namespace node
}  // namespace dai
