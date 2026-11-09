#include <catch2/catch_all.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/utility/ImageManipImpl.hpp"

namespace {
using Type = dai::ImgFrame::Type;
using Matrix = std::array<std::array<float, 3>, 3>;

struct TestImage {
    dai::impl::FrameSpecs specs;
    std::shared_ptr<dai::impl::_ImageManipMemory> memory;
    std::vector<cv::Mat> planes;

    /** Allocate the requested format and expose its storage planes for reference checks. */
    TestImage(Type type, int width, int height) : specs(dai::impl::getDstFrameSpecs(width, height, type)) {
        memory = std::make_shared<dai::impl::_ImageManipMemory>(dai::impl::getAlignedOutputFrameSize(type, width, height));
        std::fill(memory->getOffsetData().begin(), memory->getOffsetData().end(), 0xa5);
        const bool planar = type == Type::RGB888p || type == Type::RGBF16F16F16p || type == Type::BGRF16F16F16p;
        const bool yuv = type == Type::NV12 || type == Type::YUV420p;
        const bool interleaved = type == Type::RGB888i || type == Type::RGBF16F16F16i || type == Type::BGRF16F16F16i;
        const int depth = dai::impl::isFp16(type) ? CV_16F : (type == Type::RAW16 ? CV_16U : CV_8U);
        const std::array<uint32_t, 3> offsets = {specs.p1Offset, specs.p2Offset, specs.p3Offset};
        const std::array<uint32_t, 3> strides = {specs.p1Stride, specs.p2Stride, specs.p3Stride};
        const int planeCount = planar || type == Type::YUV420p ? 3 : (type == Type::NV12 ? 2 : 1);
        for(int p = 0; p < planeCount; ++p) {
            const int scale = yuv && p > 0 ? 2 : 1;
            const int channels = interleaved ? 3 : (type == Type::NV12 && p == 1 ? 2 : 1);
            planes.emplace_back(height / scale, width / scale, CV_MAKETYPE(depth, channels), memory->getOffsetData().data() + offsets[p], strides[p]);
        }
    }

    /** Fill every plane with a spatial pattern that exposes interpolation errors. */
    void fill() {
        for(size_t p = 0; p < planes.size(); ++p) {
            auto& image = planes[p];
            cv::Mat values(image.size(), CV_MAKETYPE(CV_32F, image.channels()));
            for(int y = 0; y < image.rows; ++y) {
                for(int x = 0; x < image.cols; ++x) {
                    for(int c = 0; c < image.channels(); ++c) {
                        values.ptr<float>(y)[x * image.channels() + c] =
                            image.depth() == CV_16F ? -3.f + x * 0.125f + y * 0.25f + 2 * c + p
                                                    : (image.depth() == CV_16U ? 4096.f + 3 * x + 5 * y : 30.f + x + 2 * y + 3 * c + 13 * p);
                    }
                }
            }
            values.convertTo(image, image.type());
        }
    }
};

/** Compare transformed planes with OpenCV using the same background and chroma scale. */
void checkWarp(const TestImage& input, const TestImage& output, const Matrix& matrix, const uint32_t* background, bool perspective = false) {
    REQUIRE(output.planes.size() == input.planes.size());
    for(size_t p = 0; p < input.planes.size(); ++p) {
        const auto& src = input.planes[p];
        const auto& dst = output.planes[p];
        auto planeMatrix = matrix;
        const int scale = input.specs.width / src.cols;
        planeMatrix[0][2] /= scale;
        planeMatrix[1][2] /= scale;
        const int channel = input.planes.size() == 3 || p == 1 ? static_cast<int>(p) : 0;
        const auto bg = src.channels() == 3 ? cv::Scalar(background[0], background[1], background[2])
                                            : (src.channels() == 2 ? cv::Scalar(background[1], background[2]) : cv::Scalar(background[channel]));
        cv::Mat source = src, expected, actual;
        if(src.depth() == CV_16F) src.convertTo(source, CV_32F);
        if(perspective) {
            cv::warpPerspective(source, expected, cv::Mat(3, 3, CV_32F, planeMatrix.data()), dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, bg);
        } else {
            cv::warpAffine(source, expected, cv::Mat(2, 3, CV_32F, planeMatrix.data()), dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, bg);
        }
        if(src.depth() == CV_16F) expected.convertTo(expected, CV_16F);
        expected.convertTo(expected, CV_32F);
        dst.convertTo(actual, CV_32F);
        CAPTURE(p);
        CHECK(cv::norm(actual, expected, cv::NORM_INF) <= (src.depth() == CV_16F ? 0.016 : 1.0));
    }
}

/** Map output boundaries back into source coordinates for crop construction. */
std::vector<std::array<std::array<float, 2>, 4>> sourceCorners(const Matrix& matrix, int width, int height) {
    auto copy = matrix;
    cv::Mat inverse = cv::Mat(3, 3, CV_32F, copy.data()).inv();
    std::vector<cv::Point2f> corners = {
        {0, 0}, {static_cast<float>(width), 0}, {static_cast<float>(width), static_cast<float>(height)}, {0, static_cast<float>(height)}};
    cv::perspectiveTransform(corners, corners, inverse);
    return {{{{corners[0].x, corners[0].y}, {corners[1].x, corners[1].y}, {corners[2].x, corners[2].y}, {corners[3].x, corners[3].y}}}};
}
}  // namespace

TEST_CASE("Host ImageManip horizontal translations preserve placement and background", "[imagemanip][host-warp]") {
    const auto type = GENERATE(Type::GRAY8,
                               Type::RAW16,
                               Type::RGB888i,
                               Type::RGB888p,
                               Type::NV12,
                               Type::YUV420p,
                               Type::GRAYF16,
                               Type::RGBF16F16F16i,
                               Type::BGRF16F16F16i,
                               Type::RGBF16F16F16p,
                               Type::BGRF16F16F16p);
    const int outputWidth = GENERATE(48, 64, 80);
    const float translation = GENERATE(-8.f, 0.5f, 8.f);
    CAPTURE(type, outputWidth, translation);
    TestImage input(type, 64, 48), output(type, outputWidth, 48);
    input.fill();
    const Matrix matrix = {{{1, 0, translation}, {0, 1, 0}, {0, 0, 1}}};
    dai::impl::WarpH warp;
    warp.build(input.specs, output.specs, type, matrix, sourceCorners(matrix, outputWidth, 48));
    warp.setBackgroundColor(17, 43, 91);
    warp.apply(input.memory, output.memory);
    checkWarp(input, output, matrix, warp.backgroundColor);
}

TEST_CASE("Host ImageManip rotated and perspective crops retain interpolation neighbors", "[imagemanip][host-warp]") {
    const auto type = GENERATE(Type::GRAY8,
                               Type::RAW16,
                               Type::RGB888i,
                               Type::RGB888p,
                               Type::NV12,
                               Type::YUV420p,
                               Type::GRAYF16,
                               Type::RGBF16F16F16i,
                               Type::BGRF16F16F16i,
                               Type::RGBF16F16F16p,
                               Type::BGRF16F16F16p);
    TestImage input(type, 64, 48);
    input.fill();
    for(float angle : {0.f, -7.5f, 7.5f, 18.f, 45.f, 90.f}) {
        for(bool perspective : {false, true}) {
            // Subsampled formats exercise the affine path; perspective cases
            // use full-resolution planes so the same reference applies.
            if(perspective && (type == Type::NV12 || type == Type::YUV420p)) continue;
            CAPTURE(type, angle, perspective);
            TestImage output(type, 16, 12);
            auto rotation = cv::getRotationMatrix2D(cv::Point2f(30, 22), angle, 1);
            rotation.at<double>(0, 2) += 8 - 30;
            rotation.at<double>(1, 2) += 6 - 22;
            Matrix matrix = {
                {{static_cast<float>(rotation.at<double>(0, 0)), static_cast<float>(rotation.at<double>(0, 1)), static_cast<float>(rotation.at<double>(0, 2))},
                 {static_cast<float>(rotation.at<double>(1, 0)), static_cast<float>(rotation.at<double>(1, 1)), static_cast<float>(rotation.at<double>(1, 2))},
                 {perspective ? 0.002f : 0.f, perspective ? -0.001f : 0.f, 1}}};
            dai::impl::WarpH warp;
            warp.build(input.specs, output.specs, type, matrix, sourceCorners(matrix, 16, 12));
            warp.setBackgroundColor(17, 43, 91);
            warp.apply(input.memory, output.memory);
            checkWarp(input, output, matrix, warp.backgroundColor, perspective);
        }
    }
}

TEST_CASE("Host ImageManip scaled crops retain interpolation neighbors", "[imagemanip][host-warp]") {
    const auto type = GENERATE(Type::GRAY8,
                               Type::RAW16,
                               Type::RGB888i,
                               Type::RGB888p,
                               Type::NV12,
                               Type::YUV420p,
                               Type::GRAYF16,
                               Type::RGBF16F16F16i,
                               Type::BGRF16F16F16i,
                               Type::RGBF16F16F16p,
                               Type::BGRF16F16F16p);
    const float scale = GENERATE(1.5f, 2.f, 3.f);
    CAPTURE(type, scale);
    TestImage input(type, 64, 48), output(type, 16, 12);
    input.fill();
    const Matrix matrix = {{{scale, 0, -24 * scale}, {0, scale, -18 * scale}, {0, 0, 1}}};
    dai::impl::WarpH warp;
    warp.build(input.specs, output.specs, type, matrix, sourceCorners(matrix, 16, 12));
    warp.setBackgroundColor(17, 43, 91);
    warp.apply(input.memory, output.memory);
    checkWarp(input, output, matrix, warp.backgroundColor);
}
