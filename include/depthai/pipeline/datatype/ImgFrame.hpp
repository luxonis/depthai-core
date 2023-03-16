#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

// project
#include "depthai/build/config.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai-shared/datatype/RawImgFrame.hpp"

// optional
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

class BaseTransformation {
   protected:
    RawImgTransformation rawImgTransformation;
   public:
    BaseTransformation(RawImgTransformation rawImgTransformation) : rawImgTransformation(rawImgTransformation) {};
    virtual dai::Point2f trans(dai::Point2f point) = 0;
    virtual dai::Point2f invTrans(dai::Point2f point) = 0;
    virtual ~BaseTransformation() = default;
};

class ScaleTransformation : public BaseTransformation {
    using BaseTransformation::BaseTransformation;
    dai::Point2f trans(dai::Point2f point);
    dai::Point2f invTrans(dai::Point2f point);
};

class CropTransformation : public BaseTransformation {
    using BaseTransformation::BaseTransformation;
    dai::Point2f trans(dai::Point2f point);
    dai::Point2f invTrans(dai::Point2f point);
};

class PadTransformation : public BaseTransformation {
    using BaseTransformation::BaseTransformation;
    dai::Point2f trans(dai::Point2f point);
    dai::Point2f invTrans(dai::Point2f point);
};


class FlipTransformation : public BaseTransformation {
    using BaseTransformation::BaseTransformation;
    dai::Point2f trans(dai::Point2f point);
    dai::Point2f invTrans(dai::Point2f point);
};

class RotateTransformation : public BaseTransformation {
    using BaseTransformation::BaseTransformation;
    dai::Point2f trans(dai::Point2f point);
    dai::Point2f invTrans(dai::Point2f point);
};

/**
 * ImgFrame message. Carries image data and metadata.
 */
class ImgFrame : public Buffer {
    Serialized serialize() const override;

   public:
    RawImgFrame& img;
    // Raw* mirror
    using Type = RawImgFrame::Type;
    using Specs = RawImgFrame::Specs;
    using CameraSettings = RawImgFrame::CameraSettings;

    /**
     * Construct ImgFrame message.
     * Timestamp is set to now
     */
    ImgFrame();
    ImgFrame(size_t size);
    explicit ImgFrame(std::shared_ptr<RawImgFrame> ptr);
    virtual ~ImgFrame() = default;

    std::vector<std::shared_ptr<BaseTransformation>> transformations;
    // getters
    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves image timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used mostly for debugging
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves image category
     */
    unsigned int getCategory() const;

    /**
     * Retrieves image sequence number
     */
    int64_t getSequenceNum() const;

    /**
     * Retrieves image width in pixels
     */
    unsigned int getWidth() const;

    /**
     * Retrieves image height in pixels
     */
    unsigned int getHeight() const;

    /**
     * Retrieves source image width in pixels
     */
    unsigned int getSourceWidth() const;

    /**
     * Retrieves source image height in pixels
     */
    unsigned int getSourceHeight() const;

    /**
     * Retrieve raw data for ImgFrame.
     * @returns raw image frame config
     */
    dai::RawImgFrame get() const;

    /**
     * Retrieves image type
     */
    Type getType() const;

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

    // setters
    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    ImgFrame& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgFrame& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

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
     * Specifies sequence number
     *
     * @param seq Sequence number
     */
    ImgFrame& setSequenceNum(int64_t seq);

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

    /**
     * Specifies source frame width
     *
     * @param width frame width
     */
    ImgFrame& setSourceWidth(unsigned int width);

    /**
     * Specifies source frame height
     *
     * @param height frame height
     */
    ImgFrame& setSourceHeight(unsigned int height);

    /**
     * Specifies source frame size
     *
     * @param height frame height
     * @param width frame width
     */
    ImgFrame& setSourceSize(unsigned int width, unsigned int height);

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
     * Copy over image tranformations and information about the source frame from a frame
     * @param sourceFrame source frame from which the transformations are copied from
     */
    ImgFrame& copyTransformationsFrom(std::shared_ptr<dai::ImgFrame> sourceFrame);

    /**
     * Add a flip transformation to the frame
     * This doesn't transform the image, but rather just saves the fact that it has been done.
     * @param horizontalFlip horizontal flip
     * @param verticalFlip vertical flip
     */
    ImgFrame& transformSetFlip(bool horizontalFlip, bool verticalFlip);

    /**
     * Add a padding transformation to the frame.
     * This doesn't transform the image, but rather just saves the fact that it has been done.
     *
     * Padding either has to be relative to padded image or absolute in pixels.
     *
     * @param topPadding top padding
     * @param bottomPadding bottom padding
     * @param leftPadding left padding
     * @param rightPadding right padding
     */
    ImgFrame& transformSetPadding(float topPadding, float bottomPadding, float leftPadding, float rightPadding);

    /**
     * Add a crop transformation to the frame.
     * This doesn't transform the image, but rather just saves the fact that it has been done.
     *
     * @param crop crop rectangle - can be either relative or absolute
     */
    ImgFrame& transformSetCrop(dai::Rect crop);

    /**
     * Add a rotation transformation to the frame.
     * This doesn't transform the image, but rather just saves the fact that it has been done.
     *
     * @param rotationAngle rotation angle in degrees
     * @param rotationPoint point around which the rotation was performed
     */
    ImgFrame& transformSetRotation(float rotationAngle, dai::Point2f rotationPoint = {0.5, 0.5});

    /**
     * Add a scale transformation to the frame.
     *
     * This doesn't transform the image, but rather just saves the fact that it has been done.
     * @param scaleFactorX scale factor in X direction
     * @param scaleFactorY scale factor in Y direction
     */
    ImgFrame& transformSetScale(float scaleFactorX, float scaleFactorY);

    /**
     * Transform a point from the current frame to the source frame
     * @param point point to transform
     * @returns transformed point
     */
    dai::Point2f transformPointFromSource(dai::Point2f point);

    /**
     * Transform a point from the source frame to the current frame
     * @param point point to transform
     * @returns transformed point
     */
    dai::Point2f transformPointToSource(dai::Point2f point);


    /**
     * Transform a rectangle from the source frame to the current frame
     *
     * @param rect rectangle to transform
     * @returns transformed rectangle
    */
    dai::Rect transformRectFromSource(dai::Rect rect);

    /**
     * Transform a rectangle from the current frame to the source frame
     *
     * @param rect rectangle to transform
     * @returns transformed rectangle
    */
    dai::Rect transformRectToSource(dai::Rect rect);

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
    float getSourceDFov();

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Get the source horizontal field of view
     *
     * @param degrees field of view in degrees
     */
    float getSourceHFov();

    /**
     * @note Fov API works correctly only on rectilinear frames
     * Get the source vertical field of view
     *
     * @param degrees field of view in degrees
     */
    float getSourceVFov();


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

#endif
};

}  // namespace dai
