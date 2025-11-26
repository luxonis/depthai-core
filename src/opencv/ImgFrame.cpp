#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include <cassert>
#include <cmath>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// #include "spdlog/spdlog.h"

namespace dai {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
ImgFrame& ImgFrame::setFrame(cv::Mat frame) {
    std::vector<uint8_t> dataVec;
    dataVec.reserve(frame.total() * frame.elemSize());
    if(!frame.isContinuous()) {
        for(int i = 0; i < frame.rows; i++) {
            dataVec.insert(dataVec.end(), frame.ptr(i), frame.ptr(i) + frame.cols * frame.elemSize());
        }
    } else {
        dataVec.insert(dataVec.begin(), frame.datastart, frame.dataend);
    }
    setData(dataVec);
    return *this;
}

cv::Mat ImgFrame::getFrame(bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    cv::Mat mat;
    cv::Size size = {0, 0};
    int type = 0;
    switch(getType()) {
        case Type::RGB888i:
        case Type::BGR888i:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC3;
            break;
        case Type::BGR888p:
        case Type::RGB888p:
            size = cv::Size(getWidth(), getHeight() * 3);
            type = CV_8UC1;
            break;
        case Type::YUV420p:
        case Type::NV12:
        case Type::NV21:
            size = cv::Size(getWidth(), getPlaneHeight() * 3 / 2);
            type = CV_8UC1;
            break;

        case Type::YUV422i:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC2;
            break;

        case Type::RAW8:
        case Type::GRAY8:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC1;
            break;

        case Type::GRAYF16:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16FC1;
            break;

        case Type::RAW16:
        case Type::RAW14:
        case Type::RAW12:
        case Type::RAW10:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16UC1;
            break;

        case Type::RAW32:
            size = cv::Size(getWidth(), getHeight());
            type = CV_32SC1;
            break;
        case Type::RGB161616:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16UC3;
            break;
        case Type::RGBF16F16F16i:
        case Type::BGRF16F16F16i:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16FC3;
            break;
        case Type::RGBF16F16F16p:
        case Type::BGRF16F16F16p:
            size = cv::Size(getWidth(), getHeight() * 3);
            type = CV_16FC1;
            break;

        case Type::BITSTREAM:
        default:
            size = cv::Size(static_cast<int>(getData().size()), 1);
            type = CV_8UC1;
            break;
    }

    // Check if enough data
    long requiredSize = CV_ELEM_SIZE(type) * size.area();

    // TMP TMP
    // long actualSize = static_cast<long>(img.data.size());
    long actualSize = static_cast<long>(data->getSize());

    if(actualSize < requiredSize) {
        throw std::runtime_error("ImgFrame doesn't have enough data to encode specified frame, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ". Maybe metadataOnly transfer was made?");
    } else if(actualSize > requiredSize) {
        // FIXME doesn't build on Windows (multiple definitions during link)
        // logger::warn("ImgFrame has excess data: actual {}, expected {}", actualSize, requiredSize);
    }
    if(getWidth() <= 0 || getHeight() <= 0) {
        throw std::runtime_error("ImgFrame metadata not valid (width or height = 0)");
    }

    // Copy or reference to existing data
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);

        // TMPTMP
        // Copy number of bytes that are available by Mat space or by img data size
        // std::memcpy(mat.data, img.data.data(), std::min((long)(img.data.size()), (long)(mat.dataend - mat.datastart)));
        std::memcpy(mat.data, data->getData().data(), std::min((long)(data->getSize()), (long)(mat.dataend - mat.datastart)));
        // TODO stride handling
    } else {
        // TMP TMP
        if(fb.stride == 0) {
            mat = cv::Mat(size, type, data->getData().data());
        } else {
            mat = cv::Mat(size, type, data->getData().data(), getStride());
        }
    }

    return mat;
}

cv::Mat ImgFrame::getCvFrame(cv::MatAllocator* allocator) {
    auto mergePlanarChannels = [&](int cvType, cv::Mat& output, size_t offset0, size_t offset1, size_t offset2) -> void {
        cv::Size s(getWidth(), getHeight());
        std::vector<cv::Mat> channels;
        channels.push_back(cv::Mat(s, cvType, (uint8_t*)getData().data() + offset0, getStride()));
        channels.push_back(cv::Mat(s, cvType, (uint8_t*)getData().data() + offset1, getStride()));
        channels.push_back(cv::Mat(s, cvType, (uint8_t*)getData().data() + offset2, getStride()));
        cv::merge(channels, output);
    };
    cv::Mat frame = getFrame();
    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }

    switch(getType()) {
        case Type::RGB888i:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_RGB2BGR);
            break;

        case Type::BGR888i:
            frame.copyTo(output);
            break;

        case Type::RGBF16F16F16i:
            frame.convertTo(frame, CV_32FC3);  // opencv does not support color conversions from FP16 to RGB. Need to convert to FP32 first
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_RGB2BGR);
            output.convertTo(output, CV_16FC3);
            break;

        case Type::RGB888p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            size_t offset0 = 0;
            size_t offset1 = s.area();
            size_t offset2 = s.area() * 2;
            if(fb.p1Offset != 0 || fb.p2Offset != 0 || fb.p3Offset != 0) {  // If any of the offsets are set, use all of them
                offset0 = fb.p1Offset;
                offset1 = fb.p2Offset;
                offset2 = fb.p3Offset;
            }
            mergePlanarChannels(CV_8UC1, output, offset2, offset1, offset0);  // RGB -> BGR
        } break;

        case Type::BGR888p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            size_t offset0 = 0;
            size_t offset1 = s.area();
            size_t offset2 = s.area() * 2;
            if(fb.p1Offset != 0 || fb.p2Offset != 0 || fb.p3Offset != 0) {  // If any of the offsets are set, use all of them
                offset0 = fb.p1Offset;
                offset1 = fb.p2Offset;
                offset2 = fb.p3Offset;
            }
            mergePlanarChannels(CV_8UC1, output, offset0, offset1, offset2);  //  BGR
        } break;

        case Type::RGBF16F16F16p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            size_t offset0 = 0;
            size_t offset1 = s.area() * 2;
            size_t offset2 = s.area() * 4;
            if(fb.p1Offset != 0 || fb.p2Offset != 0 || fb.p3Offset != 0) {
                offset0 = fb.p1Offset;
                offset1 = fb.p2Offset;
                offset2 = fb.p3Offset;
            }
            mergePlanarChannels(CV_16FC1, output, offset2, offset1, offset0);  // RGB -> BGR
        } break;

        case Type::BGRF16F16F16p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            size_t offset0 = 0;
            size_t offset1 = s.area() * 2;
            size_t offset2 = s.area() * 4;
            if(fb.p1Offset != 0 || fb.p2Offset != 0 || fb.p3Offset != 0) {
                offset0 = fb.p1Offset;
                offset1 = fb.p2Offset;
                offset2 = fb.p3Offset;
            }
            mergePlanarChannels(CV_16FC1, output, offset0, offset1, offset2);  // BGR
        } break;

        case Type::YUV420p:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
            break;

        case Type::YUV422i:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_YUYV);
            break;

        case Type::NV12:
        case Type::NV21: {
            int code = (getType() == Type::NV12) ? cv::ColorConversionCodes::COLOR_YUV2BGR_NV12 : cv::ColorConversionCodes::COLOR_YUV2BGR_NV21;
            cv::Size s(getWidth(), getHeight());
            int type = CV_8UC1;
            int step = getStride();
            cv::Mat frameY(s, type, getData().data(), step);
            cv::Mat frameUV(s / 2, type, getData().data() + getPlaneStride(), step);
            cv::cvtColorTwoPlane(frameY, frameUV, output, code);
        } break;

        case Type::RAW8:
        case Type::RAW16:
        case Type::RAW14:
        case Type::RAW12:
        case Type::RAW10:
        case Type::GRAY8:
        case Type::GRAYF16:
            frame.copyTo(output);
            break;

        default:
            frame.copyTo(output);
            break;
    }

    return output;
}

ImgFrame& ImgFrame::setCvFrame(cv::Mat mat, Type type) {
    cv::Mat output;
    setType(type);
    setSize(mat.cols, mat.rows);
    unsigned int size = mat.cols * mat.rows;

    auto setPlanarDataVec = [&output, size](std::vector<uint8_t>& dataVec, size_t offsetB, size_t offsetG, size_t offsetR, size_t stride, int cvType) {
        cv::Mat chB(output.rows, output.cols, cvType, dataVec.data() + size * offsetB, stride);
        cv::Mat chG(output.rows, output.cols, cvType, dataVec.data() + size * offsetG, stride);
        cv::Mat chR(output.rows, output.cols, cvType, dataVec.data() + size * offsetR, stride);
        std::vector<cv::Mat> channels = {chB, chG, chR};  // assumes mat is BGR
        cv::split(output, channels);
    };

    switch(type) {
        case Type::RGB888i: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 3;
            mat.convertTo(output, CV_8UC3);
            cv::cvtColor(output, output, cv::ColorConversionCodes::COLOR_BGR2RGB);
            setFrame(output);
        } break;

        case Type::BGR888i: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 3;
            mat.convertTo(output, CV_8UC3);
            setFrame(output);
        } break;

        case Type::RGB161616: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 6;
            mat.convertTo(output, CV_16UC3);
            cv::cvtColor(output, output, cv::ColorConversionCodes::COLOR_BGR2RGB);
            setFrame(output);
        } break;

        case Type::RGBF16F16F16i: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 6;
            mat.convertTo(output, CV_32F);  // opencv does not support color conversions from FP16 to RGB. Need to convert to FP32 first
            cv::cvtColor(output, output, cv::ColorConversionCodes::COLOR_BGR2RGB);
            output.convertTo(output, CV_16FC3);
            setFrame(output);
        } break;

        case Type::BGRF16F16F16i: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 3 * 2;
            mat.convertTo(output, CV_16FC3);
            setFrame(output);
        } break;

        case Type::RGB888p: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols;
            fb.p1Offset = 0;
            fb.p2Offset = size;
            fb.p3Offset = size * 2;
            mat.convertTo(output, CV_8UC3);
            std::vector<uint8_t> dataVec(size * 3);
            setPlanarDataVec(dataVec, 2, 1, 0, fb.stride, CV_8UC1);  // R channel is first == 0 offset
            setData(dataVec);
        } break;

        case Type::RGBF16F16F16p: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 2;
            fb.p1Offset = 0;
            fb.p2Offset = size * 2;
            fb.p3Offset = size * 4;
            std::vector<uint8_t> dataVec(size * 3 * 2);
            mat.convertTo(output, CV_16FC3);
            setPlanarDataVec(dataVec, 2 * 2, 1 * 2, 0, fb.stride, CV_16FC1);  // R channel is first == 0 offset
            setData(dataVec);
        } break;

        case Type::BGR888p: {
            fb.stride = mat.cols;
            fb.p1Offset = 0;
            fb.p2Offset = size;
            fb.p3Offset = size * 2;

            std::vector<uint8_t> dataVec(size * 3);
            mat.convertTo(output, CV_8UC3);
            setPlanarDataVec(dataVec, 0, 1, 2, fb.stride, CV_8UC1);  // B channel is first
            setData(dataVec);
        } break;

        case Type::BGRF16F16F16p: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols * 2;
            fb.p1Offset = 0;
            fb.p2Offset = size * 2;
            fb.p3Offset = size * 4;
            std::vector<uint8_t> dataVec(size * 3 * 2);
            mat.convertTo(output, CV_16FC3);
            setPlanarDataVec(dataVec, 0, 1 * 2, 2 * 2, fb.stride, CV_16FC1);  // B channel is first
            setData(dataVec);
        } break;

        case Type::YUV420p: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols;
            fb.p1Offset = 0;
            fb.p2Offset = size;
            fb.p3Offset = size / 4;
            cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_BGR2YUV_I420);
            setFrame(output);
        } break;

        case Type::NV12:
        case Type::NV21: {
            CV_Assert(mat.channels() == 3);
            fb.stride = mat.cols;
            fb.p1Offset = 0;
            fb.p2Offset = size;
            fb.p3Offset = size;
            auto code = type == Type::NV12 ? cv::COLOR_BGR2YUV_I420 : cv::COLOR_BGR2YUV_YV12;
            cv::cvtColor(mat, output, code);
            int chromaRows = mat.rows / 2;
            int chromaCols = mat.cols / 2;
            unsigned int ySize = mat.cols * mat.rows;
            assert(ySize % 4 == 0);
            unsigned int uvSize = ySize / 4;
            assert(output.isContinuous());
            assert(output.total() * output.elemSize() == mat.rows * mat.cols * 3UL / 2UL);
            assert(ySize + 2 * uvSize == output.total());

            std::vector<uint8_t> dataVec(ySize + 2 * uvSize);
            std::memcpy(dataVec.data(), output.ptr(), ySize);
            cv::Mat uvDest(chromaRows, chromaCols, CV_8UC2, dataVec.data() + ySize);
            cv::Mat chromaPlane1(chromaRows, chromaCols, CV_8UC1, output.ptr() + ySize);           // if NV12: U plane, if NV21: V plane
            cv::Mat chromaPlane2(chromaRows, chromaCols, CV_8UC1, output.ptr() + ySize + uvSize);  // if NV12: V plane, if NV21: U plane
            std::vector<cv::Mat> channels = {chromaPlane1, chromaPlane2};
            cv::merge(channels, uvDest);
            setData(dataVec);
        } break;

        case Type::RAW8:
        case Type::GRAY8: {
            fb.stride = mat.cols;
            mat.convertTo(output, CV_8UC1);
            if(output.channels() == 3) {
                cv::cvtColor(output, output, cv::ColorConversionCodes::COLOR_BGR2GRAY);
            }
            setFrame(output);
        } break;
        case Type::RAW14:
        case Type::RAW12:
        case Type::RAW10:
        case Type::RAW16: {
            fb.stride = mat.cols * 2;
            mat.convertTo(output, CV_16UC1);
            if(output.channels() == 3) {
                cv::cvtColor(output, output, cv::ColorConversionCodes::COLOR_BGR2GRAY);
            }
            setFrame(output);
        } break;

        case Type::RAW32: {
            CV_Assert(mat.channels() == 1);
            fb.stride = mat.cols * 4;
            mat.convertTo(output, CV_32SC1);
            setFrame(output);
        } break;

        case Type::GRAYF16: {
            CV_Assert(mat.channels() == 1);
            fb.stride = mat.cols * 2;
            mat.convertTo(output, CV_16FC1);
            setFrame(output);
        } break;

        default:
            setFrame(mat);
            break;
    }
    return *this;
}
#pragma GCC diagnostic pop

}  // namespace dai
