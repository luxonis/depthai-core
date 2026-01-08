#pragma once

#include <chrono>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

class EncodedFrame : public Buffer, public ProtoSerializable {
   public:
    enum class Profile : std::uint8_t { JPEG, AVC, HEVC };
    enum class FrameType : std::uint8_t { I, P, B, Unknown };
    using CameraSettings = ImgFrame::CameraSettings;

    CameraSettings cam;
    uint32_t instanceNum = 0;  // Which source created this frame (color, mono, ...)

    unsigned int width;   // width in pixels
    unsigned int height;  // height in pixels

    uint32_t quality;
    uint32_t bitrate;
    Profile profile;

    bool lossless;   // jpeg
    FrameType type;  // h264

    uint32_t frameOffset = 0;
    uint32_t frameSize = 0;

    ImgTransformation transformation;

    virtual ~EncodedFrame();

    // getters
    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves image width in pixels
     */
    unsigned int getWidth() const;

    /**
     * Retrieves image height in pixels
     */
    unsigned int getHeight() const;

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
     * Specifies frame width
     *
     * @param width frame width
     */
    EncodedFrame& setWidth(unsigned int width);

    /**
     * Specifies frame height
     *
     * @param height frame height
     */
    EncodedFrame& setHeight(unsigned int height);

    /**
     * Specifies frame size
     *
     * @param height frame height
     * @param width frame width
     */
    EncodedFrame& setSize(unsigned int width, unsigned int height);

    /**
     * Specifies frame size
     *
     * @param size frame size
     */
    EncodedFrame& setSize(std::tuple<unsigned int, unsigned int> size);

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

    ImgFrame getImgFrameMeta() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::EncodedFrame;
    }

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto(bool metadataOnly = false) const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    ProtoSerializable::SchemaPair serializeSchema() const override;
#endif

    #ifndef DEPTHAI_MESSAGES_RVC2
    DEPTHAI_SERIALIZE(EncodedFrame,
                      cam,
                      instanceNum,
                      width,
                      height,
                      quality,
                      bitrate,
                      profile,
                      lossless,
                      type,
                      frameOffset,
                      frameSize,
                      transformation,
                      Buffer::sequenceNum,
                      Buffer::ts,
                      Buffer::tsDevice,
                      Buffer::tsSystem);
    #else
    DEPTHAI_SERIALIZE(EncodedFrame,
                      cam,
                      instanceNum,
                      width,
                      height,
                      quality,
                      bitrate,
                      profile,
                      lossless,
                      type,
                      frameOffset,
                      frameSize,
                      transformation,
                      Buffer::sequenceNum,
                      Buffer::ts,
                      Buffer::tsDevice);
    #endif
};

}  // namespace dai
