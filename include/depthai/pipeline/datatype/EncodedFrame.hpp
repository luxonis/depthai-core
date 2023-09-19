#pragma once

#include <chrono>

#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai-shared/datatype/RawEncodedFrame.hpp"

// optional
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

class EncodedFrame : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawEncodedFrame& frame;

  public:
    // Raw* mirror
    using Profile = RawEncodedFrame::Profile;
    using FrameType = RawEncodedFrame::FrameType;  

    /**
     * Construct EncodedFrame message.
     * Timestamp is set to now
     */
    EncodedFrame();
    explicit EncodedFrame(std::shared_ptr<RawEncodedFrame> ptr);
    virtual ~EncodedFrame() = default;

    // getters
    /**
     * Retrieves the encoding quality
     */
    unsigned int getQuality() const;

    /**
     * Returns true if encoding is lossless (JPEG only)
    */
    bool getLossless() const;

    /**
     * Retrieves frame type (H26x only)
     */
    FrameType getFrameType() const;

    /**
     * Retrieves the encoding profile (JPEG, AVC or HEVC)
     */
    Profile getProfile() const;

    // setters
    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    EncodedFrame& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    EncodedFrame& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Specifies sequence number
     *
     * @param seq Sequence number
     */
    EncodedFrame& setSequenceNum(int64_t seq);

    /**
     * Specifies the encoding quality
     *
     * @param quality Encoding quality
     */
    EncodedFrame& setQuality(unsigned int quality);

    /**
     * Specifies if encoding is lossless (JPEG only)
     *
     * @param lossless True if lossless
     */
    EncodedFrame& setLossless(bool lossless);

    /**
     * Specifies the frame type (H26x only)
     *
     * @param type Type of h26x frame (I, P, B)
     */
    EncodedFrame& setFrameType(FrameType type);

    /**
     * Specifies the encoding profile
     *
     * @param profile Encoding profile
     */
    EncodedFrame& setProfile(Profile profile);
};

}
