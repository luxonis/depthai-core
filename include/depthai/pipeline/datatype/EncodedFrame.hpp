#pragma once

#include <chrono>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class EncodedFrame : public Buffer {
   public:
    enum class Profile : std::uint8_t { JPEG, AVC, HEVC };
    enum class FrameType : std::uint8_t { I, P, B, Unknown };
    struct CameraSettings {
        int32_t exposureTimeUs;
        int32_t sensitivityIso;
        int32_t lensPosition;
        int32_t wbColorTemp;
        float lensPositionRaw;
        DEPTHAI_SERIALIZE(CameraSettings, exposureTimeUs, sensitivityIso, lensPosition, wbColorTemp, lensPositionRaw);
    };

    CameraSettings cam;
    uint32_t instanceNum = 0;  // Which source created this frame (color, mono, ...)

    uint32_t quality;
    uint32_t bitrate;
    Profile profile;

    bool lossless;   // jpeg
    FrameType type;  // h264

    uint32_t frameOffset = 0;
    uint32_t frameSize = 0;

    virtual ~EncodedFrame() = default;

    // getters
    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;
    /**
     * Retrieves exposure time
     */
    std::chrono::microseconds getExposureTime() const;

    /**
     * Retrieves sensitivity, as an ISO value
     */
    int getSensitivity() const;

    /**
     * Retrieves white-balance color temperature of the light source, in kelvins
     */
    int getColorTemperature() const;

    /**
     * Retrieves lens position, range 0..255. Returns -1 if not available
     */
    int getLensPosition() const;

    /**
     * Retrieves lens position, range 0.0f..1.0f. Returns -1 if not available
     */
    float getLensPositionRaw() const;

    /**
     * Retrieves the encoding quality
     */
    unsigned int getQuality() const;

    /**
     * Retrieves the encoding bitrate
     */
    unsigned int getBitrate() const;

    /**
     * Returns true if encoding is lossless (JPEG only)
     */
    bool getLossless() const;

    /**
     * Retrieves frame type (H26x only)
     */
    FrameType getFrameType();

    /**
     * Retrieves the encoding profile (JPEG, AVC or HEVC)
     */
    Profile getProfile() const;

    /**
     * Instance number relates to the origin of the frame (which camera)
     *
     * @param instance Instance number
     */
    EncodedFrame& setInstanceNum(unsigned int instance);

    /**
     * Specifies the encoding quality
     *
     * @param quality Encoding quality
     */
    EncodedFrame& setQuality(unsigned int quality);

    /**
     * Specifies the encoding quality
     *
     * @param quality Encoding quality
     */
    EncodedFrame& setBitrate(unsigned int bitrate);

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

    DEPTHAI_SERIALIZE(
        EncodedFrame, cam, instanceNum, quality, bitrate, profile, lossless, type, frameOffset, frameSize, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::EncodedFrame;
    };
};

}  // namespace dai
