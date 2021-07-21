#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {

std::shared_ptr<RawBuffer> ImgFrame::serialize() const {
    return raw;
}

ImgFrame::ImgFrame() : Buffer(std::make_shared<RawImgFrame>()), img(*dynamic_cast<RawImgFrame*>(raw.get())) {
    // set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}
ImgFrame::ImgFrame(std::shared_ptr<RawImgFrame> ptr) : Buffer(std::move(ptr)), img(*dynamic_cast<RawImgFrame*>(raw.get())) {}

// helpers

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(img.ts.sec) + nanoseconds(img.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(img.tsDevice.sec) + nanoseconds(img.tsDevice.nsec)};
}
unsigned int ImgFrame::getInstanceNum() const {
    return img.instanceNum;
}
unsigned int ImgFrame::getCategory() const {
    return img.category;
}
unsigned int ImgFrame::getSequenceNum() const {
    return img.sequenceNum;
}
unsigned int ImgFrame::getWidth() const {
    return img.fb.width;
}
unsigned int ImgFrame::getHeight() const {
    return img.fb.height;
}
RawImgFrame::Type ImgFrame::getType() const {
    return img.fb.type;
}

// setters
void ImgFrame::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    img.ts.sec = duration_cast<seconds>(ts).count();
    img.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
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
    img.fb.stride = width;
}
void ImgFrame::setHeight(unsigned int height) {
    img.fb.height = height;
}
void ImgFrame::setType(RawImgFrame::Type type) {
    img.fb.type = type;
}

}  // namespace dai
