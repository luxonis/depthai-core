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
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestamp(CameraExposureOffset offset) const {
    auto ts = getTimestamp();
    auto expTime = getExposureTime();
    switch(offset) {
        case CameraExposureOffset::START:
            return ts - expTime;
        case CameraExposureOffset::MIDDLE:
            return ts - expTime / 2;
        case CameraExposureOffset::END:
        default:
            return ts;
    }
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestampDevice(CameraExposureOffset offset) const {
    auto ts = getTimestampDevice();
    auto expTime = getExposureTime();
    switch(offset) {
        case CameraExposureOffset::START:
            return ts - expTime;
        case CameraExposureOffset::MIDDLE:
            return ts - expTime / 2;
        case CameraExposureOffset::END:
        default:
            return ts;
    }
}

unsigned int ImgFrame::getInstanceNum() const {
    return img.instanceNum;
}
unsigned int ImgFrame::getCategory() const {
    return img.category;
}
int64_t ImgFrame::getSequenceNum() const {
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
std::chrono::microseconds ImgFrame::getExposureTime() const {
    return std::chrono::microseconds(img.cam.exposureTimeUs);
}
int ImgFrame::getSensitivity() const {
    return img.cam.sensitivityIso;
}
int ImgFrame::getColorTemperature() const {
    return img.cam.wbColorTemp;
}
int ImgFrame::getLensPosition() const {
    return img.cam.lensPosition;
}

// setters
ImgFrame& ImgFrame::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    img.ts.sec = duration_cast<seconds>(ts).count();
    img.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
ImgFrame& ImgFrame::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    img.tsDevice.sec = duration_cast<seconds>(ts).count();
    img.tsDevice.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
ImgFrame& ImgFrame::setInstanceNum(unsigned int instanceNum) {
    img.instanceNum = instanceNum;
    return *this;
}
ImgFrame& ImgFrame::setCategory(unsigned int category) {
    img.category = category;
    return *this;
}
ImgFrame& ImgFrame::setSequenceNum(int64_t sequenceNum) {
    img.sequenceNum = sequenceNum;
    return *this;
}
ImgFrame& ImgFrame::setWidth(unsigned int width) {
    img.fb.width = width;
    img.fb.stride = width;
    return *this;
}
ImgFrame& ImgFrame::setHeight(unsigned int height) {
    img.fb.height = height;
    return *this;
}
ImgFrame& ImgFrame::setSize(unsigned int width, unsigned int height) {
    setWidth(width);
    setHeight(height);
    return *this;
}
ImgFrame& ImgFrame::setSize(std::tuple<unsigned int, unsigned int> size) {
    setSize(std::get<0>(size), std::get<1>(size));
    return *this;
}
ImgFrame& ImgFrame::setType(RawImgFrame::Type type) {
    img.fb.type = type;
    img.fb.bytesPP = RawImgFrame::typeToBpp(img.fb.type);
    return *this;
}

}  // namespace dai
