#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class ImgFrame : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const;
    RawImgFrame& img;

   public:
    ImgFrame();
    ImgFrame(std::shared_ptr<RawImgFrame> ptr);
    ~ImgFrame() = default;

    // getters
    Timestamp getTimestamp();
    unsigned int getInstanceNum();
    unsigned int getCategory();
    unsigned int getSequenceNum();
    unsigned int getWidth();
    unsigned int getHeight();
    RawImgFrame::Type getType();

    // setters
    void setTimestamp(Timestamp ts);
    void setInstanceNum(unsigned int);
    void setCategory(unsigned int);
    void setSequenceNum(unsigned int);
    void setWidth(unsigned int);
    void setHeight(unsigned int);
    void setType(RawImgFrame::Type);
};

}  // namespace dai
