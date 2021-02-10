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
    void setFrame(cv::Mat frame);
    cv::Mat getFrame(bool deepCopy = false);
    cv::Mat getBgrFrame();
#endif
};

}  // namespace dai
