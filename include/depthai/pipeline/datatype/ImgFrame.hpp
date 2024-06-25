#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

// project
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai/common/FrameEvent.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"

// optional
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

/**
 * ImgFrame message. Carries image data and metadata.
 */
class ImgFrame : public Buffer {
   public:
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;
    enum class Type {
        YUV422i,    // interleaved 8 bit
        YUV444p,    // planar 4:4:4 format
        YUV420p,    // planar 4:2:0 format
        YUV422p,    // planar 8 bit
        YUV400p,    // 8-bit greyscale
        RGBA8888,   // RGBA interleaved stored in 32 bit word
        RGB161616,  // Planar 16 bit RGB data
        RGB888p,    // Planar 8 bit RGB data
        BGR888p,    // Planar 8 bit BGR data
        RGB888i,    // Interleaved 8 bit RGB data
        BGR888i,    // Interleaved 8 bit BGR data
        LUT2,       // 1 bit  per pixel, Lookup table (used for graphics layers)
        LUT4,       // 2 bits per pixel, Lookup table (used for graphics layers)
        LUT16,      // 4 bits per pixel, Lookup table (used for graphics layers)
        RAW16,      // save any raw type (8, 10, 12bit) on 16 bits
        RAW14,      // 14bit value in 16bit storage
        RAW12,      // 12bit value in 16bit storage
        RAW10,      // 10bit value in 16bit storage
        RAW8,
        PACK10,  // SIPP 10bit packed format
        PACK12,  // SIPP 12bit packed format
        YUV444i,
        NV12,
        NV21,
        BITSTREAM,  // used for video encoder bitstream
        HDR,
        RGBF16F16F16p,  // Planar FP16 RGB data
        BGRF16F16F16p,  // Planar FP16 BGR data
        RGBF16F16F16i,  // Interleaved FP16 RGB data
        BGRF16F16F16i,  // Interleaved FP16 BGR data
        GRAY8,          // 8 bit grayscale (1 plane)
        GRAYF16,        // FP16 grayscale (normalized)
        RAW32,          // 32 bits raw
        NONE
    };

    /**
     * Construct ImgFrame message.
     * Timestamp is set to now
     */
    ImgFrame();
    ImgFrame(long fd);
    ImgFrame(size_t size);
    virtual ~ImgFrame() = default;

    ImgTransformations transformations;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImgFrame;
    };

    // getters
    /**
     * Retrieves image timestamp (at the specified offset of exposure) related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp(CameraExposureOffset offset) const;

    /**
     * Retrieves image timestamp (at the specified offset of exposure) directly captured from device's monotonic clock,
     * not synchronized to host time. Used when monotonicity is required.
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice(CameraExposureOffset offset) const;

    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves image category
     */
    unsigned int getCategory() const;

    /**
     * Retrieves image width in pixels
     */
    unsigned int getWidth() const;

    /**
     * Retrieves image line stride in bytes
     */
    unsigned int getStride() const;

    /**
     * Retrieves image plane stride (offset to next plane) in bytes
     *
     * @param current plane index, 0 or 1
     */
    unsigned int getPlaneStride(int planeIndex = 0) const;

    /**
     * Retrieves image height in pixels
     */
    unsigned int getHeight() const;

    /**
     * Retrieves image plane height in lines
     */
    unsigned int getPlaneHeight() const;

    /**
     * Retrieves source image width in pixels
     */
    unsigned int getSourceWidth() const;

    /**
     * Retrieves source image height in pixels
     */
    unsigned int getSourceHeight() const;

    /**
     * Retrieves image type
     */
    Type getType() const;

    /**
     * Retrieves image bytes per pixel
     */
    float getBytesPerPixel() const;

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
     * Instance number relates to the origin of the frame (which camera)
     *
     * @param instance Instance number
     */
    ImgFrame& setInstanceNum(unsigned int instance);

    /**
     * @param category Image category
     */
    ImgFrame& setCategory(unsigned int category);

    /**
     * Specifies frame width
     *
     * @param width frame width
     */
    ImgFrame& setWidth(unsigned int width);

    /**
     * Specifies frame height
     *
     * @param height frame height
     */
    ImgFrame& setHeight(unsigned int height);

    /**
     * Specifies frame size
     *
     * @param height frame height
     * @param width frame width
     */
    ImgFrame& setSize(unsigned int width, unsigned int height);

    /**
     * Specifies frame size
     *
     * @param size frame size
     */
    ImgFrame& setSize(std::tuple<unsigned int, unsigned int> size);

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Specifies source frame size
     *
     * @param height frame height
     * @param width frame width
     */
    ImgFrame& setSourceSize(unsigned int width, unsigned int height);

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Specifies source frame size
     *
     * @param size frame size
     */
    ImgFrame& setSourceSize(std::tuple<unsigned int, unsigned int> size);

    /**
     * Specifies frame type, RGB, BGR, ...
     *
     * @param type Type of image
     */
    ImgFrame& setType(Type type);

    /**
     * Remap a point from the current frame to the source frame
     * @param point point to remap
     * @returns remapped point
     */
    Point2f remapPointFromSource(const Point2f& point) const;

    /**
     * Remap a point from the source frame to the current frame
     * @param point point to remap
     * @returns remapped point
     */
    Point2f remapPointToSource(const Point2f& point) const;

    /**
     * Remap a rectangle from the source frame to the current frame
     *
     * @param rect rectangle to remap
     * @returns remapped rectangle
     */
    Rect remapRectFromSource(const Rect& rect) const;

    /**
     * Remap a rectangle from the current frame to the source frame
     *
     * @param rect rectangle to remap
     * @returns remapped rectangle
     */
    Rect remapRectToSource(const Rect& rect) const;

    /**
     * Convience function to initialize meta data from another frame
     * Copies over timestamps, transformations done on the image, etc.
     * @param sourceFrame source frame from which the metadata is taken from
     */
    ImgFrame& setMetadata(const ImgFrame& sourceFrame);

    /**
     * Convience function to initialize meta data from another frame
     * Copies over timestamps, transformations done on the image, etc.
     * @param sourceFrame shared pointer to source frame from which the metadata is taken from
     */
    ImgFrame& setMetadata(const std::shared_ptr<ImgFrame>& sourceFrame);

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Set the source horizontal field of view
     *
     * @param degrees field of view in degrees
     */
    ImgFrame& setSourceHFov(float degrees);

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Get the source diagonal field of view in degrees
     *
     * @returns field of view in degrees
     */
    float getSourceDFov() const;

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Get the source horizontal field of view
     *
     * @param degrees field of view in degrees
     */
    float getSourceHFov() const;

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Get the source vertical field of view
     *
     * @param degrees field of view in degrees
     */
    float getSourceVFov() const;

    /**
     * Check that the image transformation match the image size
     *
     * @returns true if the transformations are valid
     */
    bool validateTransformations() const;

    /**
     * Remap point between two source frames
     * @param point point to remap
     * @param sourceImage source image
     * @param destImage destination image
     *
     * @returns remapped point
     */
    static Point2f remapPointBetweenSourceFrames(const Point2f& originPoint, const ImgFrame& sourceImage, const ImgFrame& destImage);

    /**
     * Remap point between two frames
     * @param originPoint point to remap
     * @param originFrame origin frame
     * @param destFrame destination frame
     *
     * @returns remapped point
     */
    static Point2f remapPointBetweenFrames(const Point2f& originPoint, const ImgFrame& originFrame, const ImgFrame& destFrame);

    /**
     * Remap rectangle between two frames
     * @param originRect rectangle to remap
     * @param originFrame origin frame
     * @param destFrame destination frame
     *
     * @returns remapped rectangle
     */
    static Rect remapRectBetweenFrames(const Rect& originRect, const ImgFrame& originFrame, const ImgFrame& destFrame);

// Optional - OpenCV support
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Copies cv::Mat data to ImgFrame buffer
     *
     * @param frame Input cv::Mat frame from which to copy the data
     */
    ImgFrame& setFrame(cv::Mat frame);

    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Retrieves data as cv::Mat with specified width, height and type
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     * @returns cv::Mat with corresponding to ImgFrame parameters
     */
    cv::Mat getFrame(bool copy = false);

    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Retrieves cv::Mat suitable for use in common opencv functions.
     * ImgFrame is converted to color BGR interleaved or grayscale depending on type.
     *
     * A copy is always made
     *
     * @returns cv::Mat for use in opencv functions
     */
    cv::Mat getCvFrame();

    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Copies cv::Mat data to the ImgFrame buffer and converts to a specific type.
     *
     * @param frame Input cv::Mat BGR frame from which to copy the data
     */
    ImgFrame& setCvFrame(cv::Mat frame, Type type);

#else

    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    ImgFrame& setFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
        return *this;
    }
    template <typename... T>
    void getFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    }
    template <typename... T>
    void getCvFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    }
    template <typename... T>
    ImgFrame& setCvFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
        return *this;
    }

#endif
   public:
    static constexpr int typeToBpp(Type type) {
        switch(type) {
            case Type::YUV422i:
                return 1;
                break;
            case Type::YUV444p:
                return 1;
                break;
            case Type::YUV420p:
                return 1;
                break;
            case Type::YUV422p:
                return 1;
                break;
            case Type::YUV400p:
                return 1;
                break;
            case Type::RGBA8888:
                return 1;
                break;
            case Type::RGB161616:
                return 2;
                break;
            case Type::RGB888p:
                return 1;
                break;
            case Type::BGR888p:
                return 1;
                break;
            case Type::RGB888i:
                return 1;
                break;
            case Type::BGR888i:
                return 1;
                break;
            case Type::RGBF16F16F16p:
                return 2;
                break;
            case Type::BGRF16F16F16p:
                return 2;
                break;
            case Type::RGBF16F16F16i:
                return 2;
                break;
            case Type::BGRF16F16F16i:
                return 2;
                break;
            case Type::GRAY8:
                return 1;
                break;
            case Type::GRAYF16:
                return 2;
                break;
            case Type::LUT2:
                return 1;
                break;
            case Type::LUT4:
                return 1;
                break;
            case Type::LUT16:
                return 1;
                break;
            case Type::RAW16:
                return 2;
                break;
            case Type::RAW14:
                return 2;
                break;
            case Type::RAW12:
                return 2;
                break;
            case Type::RAW10:
                return 2;
                break;
            case Type::RAW8:
                return 1;
                break;
            case Type::PACK10:
                return 2;
                break;
            case Type::PACK12:
                return 2;
                break;
            case Type::YUV444i:
                return 1;
                break;
            case Type::NV12:
                return 1;
                break;
            case Type::NV21:
                return 1;
                break;
            case Type::BITSTREAM:
                return 1;
                break;
            case Type::HDR:
                return 1;
                break;
            case Type::RAW32:
                return 4;
                break;
            case Type::NONE:
                return 0;
                break;
        }
        return 0;
    }

    static constexpr bool isInterleaved(Type type) {
        switch(type) {
            case Type::YUV422i:
            case Type::RGB888i:
            case Type::BGR888i:
            case Type::RGBF16F16F16i:
            case Type::BGRF16F16F16i:
            case Type::YUV444i:
                return true;
            case Type::YUV400p:
            case Type::YUV422p:
            case Type::RGB888p:
            case Type::BGR888p:
            case Type::RGBF16F16F16p:
            case Type::BGRF16F16F16p:
            case Type::YUV444p:
            case Type::YUV420p:
            case Type::RGBA8888:
            case Type::RGB161616:
            case Type::GRAY8:
            case Type::GRAYF16:
            case Type::LUT2:
            case Type::LUT4:
            case Type::LUT16:
            case Type::RAW16:
            case Type::RAW14:
            case Type::RAW12:
            case Type::RAW10:
            case Type::RAW8:
            case Type::PACK10:
            case Type::PACK12:
            case Type::NV12:
            case Type::NV21:
            case Type::BITSTREAM:
            case Type::HDR:
            case Type::RAW32:
            case Type::NONE:
                return false;
        }
        return false;
    }

    static constexpr Type toPlanar(Type type) {
        switch(type) {
            case Type::YUV422i:
                return Type::YUV422p;
            case Type::RGB888i:
                return Type::RGB888p;
            case Type::BGR888i:
                return Type::BGR888p;
            case Type::RGBF16F16F16i:
                return Type::RGBF16F16F16p;
            case Type::BGRF16F16F16i:
                return Type::BGRF16F16F16p;
            case Type::YUV444i:
                return Type::YUV444p;
            case Type::YUV400p:
            case Type::YUV422p:
            case Type::RGB888p:
            case Type::BGR888p:
            case Type::RGBF16F16F16p:
            case Type::BGRF16F16F16p:
            case Type::YUV444p:
            case Type::YUV420p:
            case Type::RGBA8888:
            case Type::RGB161616:
            case Type::GRAY8:
            case Type::GRAYF16:
            case Type::LUT2:
            case Type::LUT4:
            case Type::LUT16:
            case Type::RAW16:
            case Type::RAW14:
            case Type::RAW12:
            case Type::RAW10:
            case Type::RAW8:
            case Type::PACK10:
            case Type::PACK12:
            case Type::NV12:
            case Type::NV21:
            case Type::BITSTREAM:
            case Type::HDR:
            case Type::RAW32:
            case Type::NONE:
                return type;
        }
        return type;
    }

    static constexpr Type toInterleaved(Type type) {
        switch(type) {
            case Type::YUV422p:
                return Type::YUV422i;
            case Type::RGB888p:
                return Type::RGB888i;
            case Type::BGR888p:
                return Type::BGR888i;
            case Type::RGBF16F16F16p:
                return Type::RGBF16F16F16i;
            case Type::BGRF16F16F16p:
                return Type::BGRF16F16F16i;
            case Type::YUV444p:
                return Type::YUV444i;
            case Type::YUV400p:
            case Type::YUV420p:
            case Type::YUV422i:
            case Type::RGB888i:
            case Type::BGR888i:
            case Type::RGBF16F16F16i:
            case Type::BGRF16F16F16i:
            case Type::YUV444i:
            case Type::RGBA8888:
            case Type::RGB161616:
            case Type::GRAY8:
            case Type::GRAYF16:
            case Type::LUT2:
            case Type::LUT4:
            case Type::LUT16:
            case Type::RAW16:
            case Type::RAW14:
            case Type::RAW12:
            case Type::RAW10:
            case Type::RAW8:
            case Type::PACK10:
            case Type::PACK12:
            case Type::NV12:
            case Type::NV21:
            case Type::BITSTREAM:
            case Type::HDR:
            case Type::RAW32:
            case Type::NONE:
                return type;
        }
        return type;
    }

    struct Specs {
        Type type = Type::NONE;
        unsigned int width;     // width in pixels
        unsigned int height;    // height in pixels
        unsigned int stride;    // defined as distance in bytes from pix(y,x) to pix(y+1,x)
        unsigned int bytesPP;   // bytes per pixel (for LUT types 1)
        unsigned int p1Offset;  // Offset to first plane
        unsigned int p2Offset;  // Offset to second plane
        unsigned int p3Offset;  // Offset to third plane

        DEPTHAI_SERIALIZE(Specs, type, width, height, stride, bytesPP, p1Offset, p2Offset, p3Offset);
    };
    struct CameraSettings {
        int32_t exposureTimeUs;
        int32_t sensitivityIso;
        int32_t lensPosition;
        int32_t wbColorTemp;
        float lensPositionRaw;

        DEPTHAI_SERIALIZE(CameraSettings, exposureTimeUs, sensitivityIso, lensPosition, wbColorTemp, lensPositionRaw);
    };

    Specs fb = {};
    Specs sourceFb = {};
    CameraSettings cam;
    float HFovDegrees = 0.0;   // Horizontal field of view in degrees
    uint32_t category = 0;     //
    uint32_t instanceNum = 0;  // Which source created this frame (color, mono, ...)
    dai::FrameEvent event = dai::FrameEvent::NONE;

   public:
    DEPTHAI_SERIALIZE(ImgFrame, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, fb, sourceFb, cam, HFovDegrees, category, instanceNum, transformations);
};

}  // namespace dai
