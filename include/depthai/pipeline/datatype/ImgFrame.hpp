#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

// protected inheritance, so serialize isn't visible to users
class ImgFrame : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgFrame& img;

   public:
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

};

}  // namespace dai
