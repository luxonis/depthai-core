#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
    
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

        case dai::RawImgFrame::Type::BITSTREAM :
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

} // namespace dai
