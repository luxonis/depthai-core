#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// optional
#ifdef DEPTHAI_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

// protected inheritance, so serialize isn't visible to users
class ImgFrame : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgFrame& img;

   public:
    // Raw* mirror
    using Type = RawImgFrame::Type;
    using Specs = RawImgFrame::Specs;

    ImgFrame();
    explicit ImgFrame(std::shared_ptr<RawImgFrame> ptr);
    virtual ~ImgFrame() = default;

    // getters
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;
    unsigned int getInstanceNum() const;
    unsigned int getCategory() const;
    unsigned int getSequenceNum() const;
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    RawImgFrame::Type getType() const;

    // setters
    void setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);
    void setInstanceNum(unsigned int);
    void setCategory(unsigned int);
    void setSequenceNum(unsigned int);
    void setWidth(unsigned int);
    void setHeight(unsigned int);
    void setType(RawImgFrame::Type);

// Optional - OpenCV support
#ifdef DEPTHAI_OPENCV_SUPPORT
    /**
     * Copies cv::Mat data to ImgFrame
     * @param frame Input cv::Mat frame from which to copy the data
     */
    void setFrame(cv::Mat frame);

    /**
     * Retrieves data as cv::Mat with specified width, height and type
     * @param copy If false only a reference to data is made, otherwise a copy
     * @returns cv::Mat with corresponding to ImgFrame parameters
     */
    cv::Mat getFrame(bool copy = false);

    /**
     * Retrieves cv::Mat suitable for use in common opencv functions.
     * ImgFrame is converted to BGR or left as grayscale depending on type.
     * A copy is always made
     * @returns cv::Mat for use in opencv functions
     */
    cv::Mat getCvFrame();
#endif
};

}  // namespace dai
