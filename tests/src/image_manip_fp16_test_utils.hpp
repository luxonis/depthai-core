#pragma once

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <opencv2/opencv.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ImageManipImpl.hpp"

namespace fp16test {
using Type = dai::ImgFrame::Type;
inline const std::array<Type, 5> types = {Type::GRAYF16, Type::RGBF16F16F16i, Type::BGRF16F16F16i, Type::RGBF16F16F16p, Type::BGRF16F16F16p};

inline std::vector<cv::Mat> planes(dai::ImgFrame& frame) {
    const auto specs = dai::impl::getSrcFrameSpecs(frame.fb);
    const bool planar = frame.getType() == Type::RGBF16F16F16p || frame.getType() == Type::BGRF16F16F16p;
    const auto channels = planar || frame.getType() == Type::GRAYF16 ? 1 : 3;
    const std::array<uint32_t, 3> offsets = {specs.p1Offset, specs.p2Offset, specs.p3Offset};
    const std::array<uint32_t, 3> strides = {specs.p1Stride, specs.p2Stride, specs.p3Stride};
    std::vector<cv::Mat> result;
    for(int p = 0; p < (planar ? 3 : 1); ++p) {
        result.emplace_back(specs.height, specs.width, CV_MAKETYPE(CV_16F, channels), frame.getData().data() + offsets[p], strides[p]);
    }
    return result;
}
inline std::shared_ptr<dai::ImgFrame> makeFrame(Type type, bool padded = false) {
    constexpr int width = 23, height = 17;
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(width).setHeight(height).setType(type);
    auto specs = dai::impl::getDstFrameSpecs(width, height, type);
    if(padded) {
        specs.p1Offset = 16;
        specs.p1Stride += 14;
        specs.p2Stride = specs.p3Stride = specs.p1Stride;
        specs.p2Offset = specs.p1Offset + specs.p1Stride * height + 10;
        specs.p3Offset = specs.p2Offset + specs.p2Stride * height + 6;
    }
    frame->fb.stride = specs.p1Stride;
    frame->fb.p1Offset = specs.p1Offset;
    frame->fb.p2Offset = specs.p2Offset;
    frame->fb.p3Offset = specs.p3Offset;
    frame->setData(std::vector<uint8_t>(dai::impl::getFrameSize(type, specs), 0x5a));
    int p = 0;
    for(auto plane : planes(*frame)) {
        for(int y = 0; y < plane.rows; ++y) {
            auto* row = plane.ptr<cv::float16_t>(y);
            for(int x = 0; x < plane.cols * plane.channels(); ++x) row[x] = cv::float16_t(-8.5f + x * 0.25f + y * 0.125f + p * 10.f);
        }
        ++p;
    }
    return frame;
}
inline void checkWarp(dai::ImgFrame& output,
                      dai::ImgFrame& input,
                      const std::array<std::array<float, 3>, 3>& matrix,
                      const std::array<uint32_t, 3>& background) {
    auto src = planes(input);
    auto dst = planes(output);
    REQUIRE(src.size() == dst.size());
    for(size_t p = 0; p < src.size(); ++p) {
        cv::Mat source, expected, actual;
        src[p].convertTo(source, CV_32F);
        dst[p].convertTo(actual, CV_32F);
        const auto bg = src.size() == 3 ? cv::Scalar(background[p]) : cv::Scalar(background[0], background[1], background[2]);
        cv::warpPerspective(
            source, expected, cv::Mat(3, 3, CV_32F, const_cast<float*>(matrix[0].data())), actual.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, bg);
        // Quarter-pixel transforms have exact OpenCV table coordinates. Allow
        // the single FP16 rounding at the output. Perspective sampling also
        // allows OpenCV interpolation-table quantization at the border.
        CHECK(cv::norm(actual, expected, cv::NORM_INF) <= (matrix[2][0] == 0 ? 0.032 : 0.35));
    }
}
}  // namespace fp16test
