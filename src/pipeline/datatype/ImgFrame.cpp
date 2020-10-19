#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImgFrame::serialize() const {
    return raw;
}

ImgFrame::ImgFrame() : Buffer(std::make_shared<RawImgFrame>()), img(*dynamic_cast<RawImgFrame*>(raw.get())) {}
ImgFrame::ImgFrame(std::shared_ptr<RawImgFrame> ptr) : Buffer(ptr), img(*dynamic_cast<RawImgFrame*>(raw.get())) {}

// helpers

// getters
Timestamp ImgFrame::getTimestamp() {
    return img.ts;
}
unsigned int ImgFrame::getInstanceNum() {
    return img.instanceNum;
}
unsigned int ImgFrame::getCategory() {
    return img.category;
}
unsigned int ImgFrame::getSequenceNum() {
    return img.sequenceNum;
}
unsigned int ImgFrame::getWidth() {
    return img.fb.width;
}
unsigned int ImgFrame::getHeight() {
    return img.fb.height;
}
RawImgFrame::Type ImgFrame::getType() {
    return img.fb.type;
}

// setters
void ImgFrame::setTimestamp(Timestamp ts) {
    img.ts = ts;
}
void ImgFrame::setInstanceNum(unsigned int instanceNum) {
    img.instanceNum = instanceNum;
}
void ImgFrame::setCategory(unsigned int category) {
    img.category = category;
}
void ImgFrame::setSequenceNum(unsigned int sequenceNum) {
    img.sequenceNum = sequenceNum;
}
void ImgFrame::setWidth(unsigned int width) {
    img.fb.width = width;
}
void ImgFrame::setHeight(unsigned int height) {
    img.fb.height = height;
}
void ImgFrame::setType(RawImgFrame::Type type) {
    img.fb.type = type;
}

}  // namespace dai
