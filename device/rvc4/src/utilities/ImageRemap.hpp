#pragma once

#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace gate {

inline cv::Mat planeView(ImgFrame& frame, size_t offset, int rows, int cols, int type, size_t stride) {
    auto data = frame.getData();
    const size_t rowBytes = static_cast<size_t>(cols) * CV_ELEM_SIZE(type);
    if(rows <= 0 || cols <= 0 || stride < rowBytes || offset > data.size() || rowBytes > data.size() - offset
       || static_cast<size_t>(rows - 1) > (data.size() - offset - rowBytes) / stride) {
        throw std::runtime_error("Align image plane exceeds its buffer or has an invalid stride");
    }
    return cv::Mat(rows, cols, type, data.data() + offset, stride);
}

inline cv::Mat yuvPlaneView(ImgFrame& frame, int plane) {
    const size_t stride = frame.getStride();
    if(plane == 0) {
        return planeView(frame, frame.fb.p1Offset, frame.getHeight(), frame.getWidth(), CV_8UC1, stride);
    }
    const bool interleaved = frame.getType() == ImgFrame::Type::NV12;
    if(!interleaved && stride % 2) throw std::runtime_error("Align requires an even YUV420p stride");
    const size_t second = frame.fb.p2Offset ? frame.fb.p2Offset : frame.fb.p1Offset + stride * frame.getHeight();
    const size_t third = frame.fb.p3Offset ? frame.fb.p3Offset : second + stride / 2 * (frame.getHeight() / 2);
    return planeView(frame,
                     plane == 1 ? second : third,
                     frame.getHeight() / 2,
                     frame.getWidth() / 2,
                     interleaved ? CV_8UC2 : CV_8UC1,
                     interleaved ? stride : stride / 2);
}

inline void remapFrame(ImgFrame& input,
                       ImgFrame& output,
                       const cv::Mat& mapX,
                       const cv::Mat& mapY,
                       cv::InterpolationFlags interpolation = cv::INTER_LINEAR) {
    if(input.getType() != ImgFrame::Type::NV12 && input.getType() != ImgFrame::Type::YUV420p) {
        cv::remap(input.getFrame(), output.getFrame(), mapX, mapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        return;
    }

    CV_Assert(input.getWidth() % 2 == 0 && input.getHeight() % 2 == 0 && output.getWidth() % 2 == 0 && output.getHeight() % 2 == 0);
    auto inputY = yuvPlaneView(input, 0);
    auto outputY = yuvPlaneView(output, 0);
    cv::remap(inputY, outputY, mapX, mapY, interpolation, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::Mat chromaX, chromaY;
    const cv::Size chromaSize(output.getWidth() / 2, output.getHeight() / 2);
    cv::resize(mapX, chromaX, chromaSize, 0, 0, cv::INTER_NEAREST);
    cv::resize(mapY, chromaY, chromaSize, 0, 0, cv::INTER_NEAREST);
    chromaX *= 0.5f;
    chromaY *= 0.5f;
    const bool interleaved = input.getType() == ImgFrame::Type::NV12;
    for(int plane = 1; plane <= (interleaved ? 1 : 2); ++plane) {
        auto inputChroma = yuvPlaneView(input, plane);
        auto outputChroma = yuvPlaneView(output, plane);
        cv::remap(inputChroma, outputChroma, chromaX, chromaY, interpolation, cv::BORDER_CONSTANT, cv::Scalar(128, 128));
    }
}

}  // namespace gate
}  // namespace dai

