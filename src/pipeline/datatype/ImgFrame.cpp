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
    return std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>{seconds(img.ts.sec) + nanoseconds(img.ts.nsec)};
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

// Optional - OpenCV support
#ifdef DEPTHAI_OPENCV_SUPPORT
void ImgFrame::setFrame(cv::Mat frame) {
    img.data.clear();
    img.data.insert(img.data.begin(), frame.datastart, frame.dataend);
}

cv::Mat ImgFrame::getFrame(bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    cv::Mat mat;
    cv::Size size = {0, 0};
    int type = 0;

    switch(getType()) {
        case Type::RGB888i:
        case Type::BGR888i:
        case Type::BGR888p:
        case Type::RGB888p:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC3;
            break;

        case Type::YUV420p:
        case Type::NV12:
        case Type::NV21:
            size = cv::Size(getWidth(), getHeight() * 3 / 2);
            type = CV_8UC1;
            break;

        case Type::RAW8:
        case Type::GRAY8:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC1;
            break;

        case Type::RGBF16F16F16i:
        case Type::BGRF16F16F16i:
        case Type::RGBF16F16F16p:
        case Type::BGRF16F16F16p:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16FC3;
            break;

        default:
            size = cv::Size(getData().size(), 1);
            type = CV_8UC1;
            break;
    }

    // Copy or reference to existing data
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);
        // Copy number of bytes as available by Mat
        std::memcpy(mat.data, img.data.data(), mat.dataend - mat.datastart);
    } else {
        // Create a reference
        mat = cv::Mat(size, type, img.data.data());
    }

    return mat;
}

cv::Mat ImgFrame::getBgrFrame() {
    cv::Mat frame = getFrame();
    cv::Mat output;

    switch(getType()) {
        case Type::RGB888i:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_RGB2BGR);
            break;

        case Type::BGR888i:
            output = frame.clone();
            break;

        case Type::YUV420p:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV420p2BGR);
            break;

        case Type::NV12:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
            break;

        case Type::NV21:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV21);
            break;

        case Type::RAW8:
        case Type::GRAY8:
            output = frame.clone();
            break;

        default:
            output = frame.clone();
            break;
    }

    return output;
}

#endif

}  // namespace dai
