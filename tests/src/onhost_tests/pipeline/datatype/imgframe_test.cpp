#include <opencv2/core/cvdef.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <vector>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "depthai/pipeline/datatype/ImgFrame.hpp"

using namespace std::chrono_literals;

namespace {

std::vector<uint8_t> makeSequential(std::size_t count, uint8_t start = 0) {
    std::vector<uint8_t> out(count);
    std::iota(out.begin(), out.end(), start);
    return out;
}

}  // namespace

TEST_CASE("ImgFrame basic metadata and layout", "[ImgFrame][Geometry]") {
    dai::ImgFrame frame;

    frame.setInstanceNum(3).setCategory(7).setSize(320, 240).setType(dai::ImgFrame::Type::RGB888i);

    REQUIRE(frame.getInstanceNum() == 3);
    REQUIRE(frame.getCategory() == 7);
    REQUIRE(frame.getWidth() == 320);
    REQUIRE(frame.getHeight() == 240);
    REQUIRE(frame.getBytesPerPixel() == Catch::Approx(3.0F));
    REQUIRE(frame.getStride() == 320 * 3);
    REQUIRE(frame.getPlaneStride() == frame.getStride() * frame.getHeight());
    REQUIRE(frame.getPlaneHeight() == frame.getHeight());

    SECTION("Custom stride and plane offsets") {
        frame.setStride(1024);
        frame.fb.p1Offset = 0;
        frame.fb.p2Offset = 100;
        frame.fb.p3Offset = 260;

        REQUIRE(frame.getStride() == 1024);
        REQUIRE(frame.getPlaneStride(0) == 100);
        REQUIRE(frame.getPlaneStride(1) == 160);
        REQUIRE(frame.getPlaneHeight() == frame.getPlaneStride() / frame.getStride());
    }

    SECTION("Size setters maintain width/height") {
        frame.setSize(640, 480);
        REQUIRE(frame.getWidth() == 640);
        REQUIRE(frame.getHeight() == 480);

        frame.setSize(std::make_tuple(1280U, 720U));
        REQUIRE(frame.getWidth() == 1280);
        REQUIRE(frame.getHeight() == 720);
    }

    SECTION("Source size sets stride and transformation") {
        frame.setSourceSize(800, 600);
        REQUIRE(frame.getSourceWidth() == 800);
        REQUIRE(frame.getSourceHeight() == 600);
        REQUIRE_FALSE(frame.validateTransformations());

        frame.setSize(800, 600);
        REQUIRE(frame.validateTransformations());
    }

    SECTION("Type changes recompute bytes per pixel and respect manual stride") {
        frame.setWidth(10).setHeight(1).setStride(0).setType(dai::ImgFrame::Type::RAW8);
        REQUIRE(frame.getBytesPerPixel() == Catch::Approx(1.0F));
        REQUIRE(frame.getStride() == 10);

        frame.setType(dai::ImgFrame::Type::RAW16);
        REQUIRE(frame.getBytesPerPixel() == Catch::Approx(2.0F));
        REQUIRE(frame.getStride() == 20);

        frame.setStride(64).setType(dai::ImgFrame::Type::RGB888i);
        REQUIRE(frame.getBytesPerPixel() == Catch::Approx(3.0F));
        REQUIRE(frame.getStride() == 64);
    }
}

TEST_CASE("ImgFrame constructors initialize buffers", "[ImgFrame][Constructors]") {
    dai::ImgFrame defaultFrame;
    REQUIRE(defaultFrame.getData().empty());

    dai::ImgFrame sized(size_t(16));
    REQUIRE(sized.getData().size() == 16);
    auto sizedData = sized.getData();
    std::fill(sizedData.begin(), sizedData.end(), 0xAB);
    REQUIRE(sized.getData()[0] == 0xAB);
}

TEST_CASE("ImgFrame timestamps and camera settings", "[ImgFrame][Timestamps]") {
    dai::ImgFrame frame;
    auto now = std::chrono::steady_clock::now();
    frame.cam.exposureTimeUs = 2000;
    frame.cam.sensitivityIso = 400;
    frame.cam.wbColorTemp = 5500;
    frame.cam.lensPosition = 120;
    frame.cam.lensPositionRaw = 0.42F;

    frame.setTimestamp(now);
    frame.setTimestampDevice(now + 5ms);

    REQUIRE(frame.getExposureTime() == 2000us);
    REQUIRE(frame.getSensitivity() == 400);
    REQUIRE(frame.getColorTemperature() == 5500);
    REQUIRE(frame.getLensPosition() == 120);
    REQUIRE(frame.getLensPositionRaw() == Catch::Approx(0.42F));

    REQUIRE(frame.getTimestamp(dai::CameraExposureOffset::END) == frame.getTimestamp());
    REQUIRE(frame.getTimestamp(dai::CameraExposureOffset::START) + frame.getExposureTime() == frame.getTimestamp());
    REQUIRE(frame.getTimestamp(dai::CameraExposureOffset::MIDDLE) + frame.getExposureTime() / 2 == frame.getTimestamp());

    REQUIRE(frame.getTimestampDevice(dai::CameraExposureOffset::END) == frame.getTimestampDevice());
    REQUIRE(frame.getTimestampDevice(dai::CameraExposureOffset::START) + frame.getExposureTime() == frame.getTimestampDevice());
}

TEST_CASE("ImgFrame type helpers", "[ImgFrame][Types]") {
    using dai::ImgFrame;
    using Type = ImgFrame::Type;

    const std::vector<std::pair<Type, int>> bppExpectations = {
        {Type::YUV422i, 1},   {Type::YUV444p, 1}, {Type::YUV420p, 1},       {Type::YUV422p, 1},       {Type::YUV400p, 1},       {Type::RGBA8888, 1},
        {Type::RGB161616, 2}, {Type::RGB888p, 1}, {Type::BGR888p, 1},       {Type::RGB888i, 3},       {Type::BGR888i, 3},       {Type::LUT2, 1},
        {Type::LUT4, 1},      {Type::LUT16, 1},   {Type::RAW16, 2},         {Type::RAW14, 2},         {Type::RAW12, 2},         {Type::RAW10, 2},
        {Type::RAW8, 1},      {Type::PACK10, 2},  {Type::PACK12, 2},        {Type::YUV444i, 1},       {Type::NV12, 1},          {Type::NV21, 1},
        {Type::BITSTREAM, 1}, {Type::HDR, 1},     {Type::RGBF16F16F16p, 2}, {Type::BGRF16F16F16p, 2}, {Type::RGBF16F16F16i, 2}, {Type::BGRF16F16F16i, 2},
        {Type::GRAY8, 1},     {Type::GRAYF16, 2}, {Type::RAW32, 4},         {Type::NONE, 0},
    };

    for(const auto& [type, expectedBpp] : bppExpectations) {
        REQUIRE(ImgFrame::typeToBpp(type) == expectedBpp);
    }

    const std::vector<std::pair<Type, bool>> interleavedExpectations = {
        {Type::YUV422i, true},        {Type::RGB888i, true},        {Type::BGR888i, true},  {Type::RGBF16F16F16i, true}, {Type::BGRF16F16F16i, true},
        {Type::YUV444i, true},        {Type::YUV400p, false},       {Type::YUV422p, false}, {Type::RGB888p, false},      {Type::BGR888p, false},
        {Type::RGBF16F16F16p, false}, {Type::BGRF16F16F16p, false}, {Type::YUV444p, false}, {Type::YUV420p, false},      {Type::RGBA8888, false},
        {Type::RGB161616, false},     {Type::GRAY8, false},         {Type::GRAYF16, false}, {Type::LUT2, false},         {Type::LUT4, false},
        {Type::LUT16, false},         {Type::RAW16, false},         {Type::RAW14, false},   {Type::RAW12, false},        {Type::RAW10, false},
        {Type::RAW8, false},          {Type::PACK10, false},        {Type::PACK12, false},  {Type::NV12, false},         {Type::NV21, false},
        {Type::BITSTREAM, false},     {Type::HDR, false},           {Type::RAW32, false},   {Type::NONE, false},
    };

    for(const auto& [type, expected] : interleavedExpectations) {
        REQUIRE(ImgFrame::isInterleaved(type) == expected);
    }

    REQUIRE(ImgFrame::toPlanar(Type::RGB888i) == Type::RGB888p);
    REQUIRE(ImgFrame::toPlanar(Type::BGR888i) == Type::BGR888p);
    REQUIRE(ImgFrame::toPlanar(Type::YUV444i) == Type::YUV444p);
    REQUIRE(ImgFrame::toPlanar(Type::YUV422i) == Type::YUV422p);
    REQUIRE(ImgFrame::toPlanar(Type::NV12) == Type::NV12);

    REQUIRE(ImgFrame::toInterleaved(Type::RGB888p) == Type::RGB888i);
    REQUIRE(ImgFrame::toInterleaved(Type::BGR888p) == Type::BGR888i);
    REQUIRE(ImgFrame::toInterleaved(Type::YUV444p) == Type::YUV444i);
    REQUIRE(ImgFrame::toInterleaved(Type::YUV422p) == Type::YUV422i);
    REQUIRE(ImgFrame::toInterleaved(Type::RAW8) == Type::RAW8);
}

TEST_CASE("ImgFrame metadata copy, clone, and data handling", "[ImgFrame][Copy]") {
    using dai::ImgFrame;
    ImgFrame source;
    source.setInstanceNum(1).setCategory(9).setType(ImgFrame::Type::BGR888i).setSize(4, 2).setSourceSize(4, 2);
    source.cam.exposureTimeUs = 1500;
    source.cam.sensitivityIso = 200;
    source.cam.wbColorTemp = 4500;
    source.cam.lensPosition = 33;
    source.cam.lensPositionRaw = 0.25F;
    source.fb.stride = 12;

    auto now = std::chrono::steady_clock::now();
    source.setTimestamp(now);
    source.setTimestampDevice(now + 5ms);

    auto srcData = makeSequential(8, 5);
    source.setData(srcData);

    ImgFrame target;
    auto* originalDataPtr = target.data.get();
    target.setMetadata(source);
    REQUIRE(target.data.get() == originalDataPtr);
    REQUIRE(target.getInstanceNum() == source.getInstanceNum());
    REQUIRE(target.getCategory() == source.getCategory());
    REQUIRE(target.getWidth() == source.getWidth());
    REQUIRE(target.cam.exposureTimeUs == source.cam.exposureTimeUs);
    REQUIRE(target.getTimestamp() == source.getTimestamp());
    REQUIRE(target.getTimestampDevice() == source.getTimestampDevice());

    REQUIRE_NOTHROW(target.setMetadata(std::make_shared<ImgFrame>(source)));
    REQUIRE_THROWS_AS(target.setMetadata(std::shared_ptr<ImgFrame>{}), std::invalid_argument);

    target.copyDataFrom(source);
    REQUIRE(target.getData().size() == source.getData().size());
    REQUIRE(std::vector<uint8_t>(target.getData().begin(), target.getData().end()) == std::vector<uint8_t>(source.getData().begin(), source.getData().end()));

    ImgFrame sharedCopy;
    sharedCopy.copyDataFrom(std::make_shared<ImgFrame>(source));
    REQUIRE(std::vector<uint8_t>(sharedCopy.getData().begin(), sharedCopy.getData().end())
            == std::vector<uint8_t>(source.getData().begin(), source.getData().end()));

    auto clone = source.clone();
    REQUIRE(clone->getWidth() == source.getWidth());
    REQUIRE(clone->getHeight() == source.getHeight());
    REQUIRE(clone->getInstanceNum() == source.getInstanceNum());
    REQUIRE(clone->getData().size() == source.getData().size());
    REQUIRE(clone->data.get() != source.data.get());

    auto cloneData = clone->getData();
    cloneData[0] = 0xFF;
    REQUIRE(source.getData()[0] == srcData[0]);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("ImgFrame OpenCV support", "[ImgFrame][OpenCV]") {
    using dai::ImgFrame;

    const int rows = 16;
    const int cols = 16;

    auto makePattern = [&](int type, bool continuous) {
        cv::Mat big(rows + 50, cols + 50, type);
        for(int r = 0; r < big.rows; ++r) {
            for(int c = 0; c < big.cols; ++c) {
                if(CV_MAT_CN(type) == 3) {
                    switch(CV_MAT_DEPTH(type)) {
                        case CV_8U:
                            big.at<cv::Vec3b>(r, c) = cv::Vec3b{
                                static_cast<uint8_t>((r + c) % 256), static_cast<uint8_t>((r * 2 + c) % 256), static_cast<uint8_t>((r + 2 * c) % 256)};
                            break;
                        case CV_16U:
                            big.at<cv::Vec3w>(r, c) =
                                cv::Vec3w{static_cast<uint16_t>((r + c) * 10), static_cast<uint16_t>((r * 2 + c) * 5), static_cast<uint16_t>((r + 2 * c) * 3)};
                            break;
                        case CV_32S:
                            big.at<cv::Vec3i>(r, c) = cv::Vec3i{r * 100 + c, r * 200 + c * 2, r * 300 + c * 3};
                            break;
                        case CV_32F:
                            big.at<cv::Vec3f>(r, c) = cv::Vec3f{
                                static_cast<float>((r + c) / 10.0F), static_cast<float>((r * 2 + c) / 20.0F), static_cast<float>((r + 2 * c) / 30.0F)};
                            break;
                        default:
                            break;
                    }
                } else {
                    switch(CV_MAT_DEPTH(type)) {
                        case CV_8U:
                            big.at<uint8_t>(r, c) = static_cast<uint8_t>((r + c) % 256);
                            break;
                        case CV_16U:
                            big.at<uint16_t>(r, c) = static_cast<uint16_t>((r + c) * 4);
                            break;
                        case CV_32S:
                            big.at<int32_t>(r, c) = r * 50 + c;
                            break;
                        case CV_32F:
                            big.at<float>(r, c) = static_cast<float>((r + c) / 15.0F);
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        cv::Mat roi = big(cv::Rect(3, 2, cols, rows));
        return continuous ? roi.clone() : roi;  // clone makes it continuous, otherwise ROI keeps a stride
    };

    auto matsClose = [](const cv::Mat& a, const cv::Mat& b, double tol) {
        if(a.size() != b.size() || a.type() != b.type()) return false;

        cv::Mat a64, b64;
        a.convertTo(a64, CV_MAKETYPE(CV_64F, a.channels()));
        b.convertTo(b64, CV_MAKETYPE(CV_64F, b.channels()));

        cv::Mat diff;
        cv::absdiff(a64, b64, diff);

        int mismatches = 0;
        if(diff.channels() == 1) {
            cv::Mat mask;
            cv::compare(diff, tol, mask, cv::CmpTypes::CMP_GT);
            mismatches = cv::countNonZero(mask);
        } else {
            std::vector<cv::Mat> channels;
            cv::split(diff, channels);
            for(const auto& c : channels) {
                cv::Mat mask;
                cv::compare(c, tol, mask, cv::CmpTypes::CMP_GT);
                mismatches += cv::countNonZero(mask);
            }
        }
        return mismatches == 0;
    };

    SECTION("getFrame throws when data is insufficient") {
        ImgFrame frame;
        frame.setType(ImgFrame::Type::BGR888i).setSize(4, 4);
        frame.setData(std::vector<uint8_t>(4));
        REQUIRE_THROWS_AS(frame.getFrame(), std::runtime_error);
    }

    SECTION("setCvFrame enforces channel requirements") {
        cv::Mat mono8(2, 2, CV_8UC1, cv::Scalar(1));
        cv::Mat bgr8(2, 2, CV_8UC3, cv::Scalar(1, 2, 3));

        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(mono8, ImgFrame::Type::RGB888i), cv::Exception);
        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(mono8, ImgFrame::Type::NV12), cv::Exception);
        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(mono8, ImgFrame::Type::YUV420p), cv::Exception);
        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(bgr8, ImgFrame::Type::RAW32), cv::Exception);
        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(bgr8, ImgFrame::Type::GRAYF16), cv::Exception);
    }

    SECTION("BGR888i converts float to UINT8 with clamping") {
        cv::Mat bgr16f(1, 1, CV_32FC3);
        bgr16f.at<cv::Vec3f>(0, 0) = cv::Vec3f(1.1F, -1.8F, 300.1F);

        ImgFrame frame;
        auto bgrCopy = bgr16f.clone();
        REQUIRE_NOTHROW(frame.setCvFrame(bgr16f, ImgFrame::Type::BGR888i));
        REQUIRE(matsClose(bgr16f, bgrCopy, 0.1));  // ensure input not modified

        auto restored = frame.getFrame();
        REQUIRE(restored.type() == CV_8UC3);
        auto pix = restored.at<cv::Vec<uint8_t, 3>>(0, 0);
        REQUIRE(pix == cv::Vec<uint8_t, 3>(1, 0, 255));
    }

    SECTION("RAW32 requires single-channel input and preserves values") {
        cv::Mat single32s = (cv::Mat_<int32_t>(1, 3) << INT32_MIN, -1, UINT32_MAX - 1);
        ImgFrame frame;
        auto singleCopy = single32s.clone();
        frame.setCvFrame(single32s, ImgFrame::Type::RAW32).setSourceSize(single32s.cols, single32s.rows);
        REQUIRE(matsClose(single32s, singleCopy, 0.1));

        auto restored = frame.getFrame();
        REQUIRE(restored.type() == CV_32SC1);
        REQUIRE(restored.rows == single32s.rows);
        REQUIRE(restored.cols == single32s.cols);
        REQUIRE(cv::countNonZero(restored != single32s) == 0);

        cv::Mat bgr8(1, 1, CV_8UC3, cv::Scalar(1, 2, 3));
        REQUIRE_THROWS_AS(ImgFrame().setCvFrame(bgr8, ImgFrame::Type::RAW32), cv::Exception);
    }

    SECTION("Planar offsets are handled correctly") {
        ImgFrame frame;
        frame.setType(ImgFrame::Type::RGB888p).setSize(2, 1).setStride(2);
        frame.fb.p1Offset = 10;  // R
        frame.fb.p2Offset = 20;  // G
        frame.fb.p3Offset = 30;  // B

        std::vector<uint8_t> data(40, 0);
        data[frame.fb.p1Offset + 0] = 5;
        data[frame.fb.p1Offset + 1] = 6;
        data[frame.fb.p2Offset + 0] = 3;
        data[frame.fb.p2Offset + 1] = 4;
        data[frame.fb.p3Offset + 0] = 1;
        data[frame.fb.p3Offset + 1] = 2;
        frame.setData(data);

        auto restored = frame.getCvFrame();
        REQUIRE(restored.type() == CV_8UC3);
        REQUIRE(restored.rows == 1);
        REQUIRE(restored.cols == 2);
        auto p0 = restored.at<cv::Vec3b>(0, 0);
        auto p1 = restored.at<cv::Vec3b>(0, 1);
        REQUIRE(p0[0] == 1);
        REQUIRE(p0[1] == 3);
        REQUIRE(p0[2] == 5);
        REQUIRE(p1[0] == 2);
        REQUIRE(p1[1] == 4);
        REQUIRE(p1[2] == 6);
    }

    SECTION("Planar offsets default to sequential planes") {
        ImgFrame frame;
        frame.setType(ImgFrame::Type::RGB888p).setSize(2, 1).setStride(2);
        frame.fb.p1Offset = 0;
        frame.fb.p2Offset = 0;
        frame.fb.p3Offset = 0;

        // Default layout: R plane (2 bytes), G plane (2 bytes), B plane (2 bytes)
        std::vector<uint8_t> data = {
            7,
            8,  // R
            9,
            10,  // G
            11,
            12  // B
        };
        frame.setData(data);

        auto restored = frame.getCvFrame();
        REQUIRE(restored.type() == CV_8UC3);
        REQUIRE(restored.rows == 1);
        REQUIRE(restored.cols == 2);
        auto p0 = restored.at<cv::Vec3b>(0, 0);
        auto p1 = restored.at<cv::Vec3b>(0, 1);
        REQUIRE(p0[0] == 11);
        REQUIRE(p0[1] == 9);
        REQUIRE(p0[2] == 7);
        REQUIRE(p1[0] == 12);
        REQUIRE(p1[1] == 10);
        REQUIRE(p1[2] == 8);
    }

    const auto contiguousBgr = makePattern(CV_8UC3, true);
    const auto nonContiguousBgr = makePattern(CV_8UC3, false);

    SECTION("RGB / BGR 888 interleaved") {
        for(const auto& src : {contiguousBgr, nonContiguousBgr}) {
            for(auto type : {ImgFrame::Type::BGR888i, ImgFrame::Type::RGB888i}) {
                ImgFrame frame;
                auto srcCopy = src.clone();
                frame.setCvFrame(src, type).setSourceSize(cols, rows);
                REQUIRE(matsClose(src, srcCopy, 0.1));

                REQUIRE(frame.getType() == type);
                auto restored = frame.getCvFrame();
                REQUIRE(restored.type() == CV_8UC3);
                REQUIRE(matsClose(restored, src.clone(), 2.0));
            }
        }
    }

    SECTION("RGB/BGR 888 planar") {
        for(const auto& src : {contiguousBgr, nonContiguousBgr}) {
            for(auto type : {ImgFrame::Type::RGB888p, ImgFrame::Type::BGR888p}) {
                ImgFrame frame;
                auto srcCopy = src.clone();
                frame.setCvFrame(src, type).setSourceSize(cols, rows);
                REQUIRE(matsClose(src, srcCopy, 0.1));

                auto planar = frame.getFrame();
                REQUIRE(planar.type() == CV_8UC1);
                REQUIRE(planar.cols == cols);
                REQUIRE(planar.rows == rows * 3);

                auto restored = frame.getCvFrame();
                REQUIRE(restored.type() == CV_8UC3);
                REQUIRE(matsClose(restored, src.clone(), 2.0));
            }
        }
    }

    SECTION("F16F16F16 i/p") {
        cv::Mat continuousBGRFP32 = makePattern(CV_32FC3, true);
        continuousBGRFP32.convertTo(continuousBGRFP32, CV_16FC3);
        cv::Mat nonContiguousBGRFP32 = makePattern(CV_32FC3, false);
        nonContiguousBGRFP32.convertTo(nonContiguousBGRFP32, CV_16FC3);

        auto typesList = {
            ImgFrame::Type::RGBF16F16F16i,
            ImgFrame::Type::BGRF16F16F16i,
            ImgFrame::Type::RGBF16F16F16p,
            ImgFrame::Type::BGRF16F16F16p,
        };

        for(cv::Mat mat : {continuousBGRFP32, nonContiguousBGRFP32}) {
            for(auto type : typesList) {
                ImgFrame frame;
                auto matCopy = mat.clone();
                frame.setCvFrame(mat, type);
                frame.setSourceSize(cols, rows);
                REQUIRE(matsClose(mat, matCopy, 0.1));

                cv::Mat restored = frame.getCvFrame();
                REQUIRE(restored.type() == CV_16FC3);
                REQUIRE(matsClose(restored, mat, 1e-3));
            }
        }
    }

    SECTION("YUV and NV formats convert back to BGR") {
        for(const auto& src : {contiguousBgr, nonContiguousBgr}) {
            for(auto type : {ImgFrame::Type::YUV420p, ImgFrame::Type::NV12, ImgFrame::Type::NV21}) {
                ImgFrame frame;
                auto srcCopy = src.clone();
                frame.setCvFrame(src, type).setSourceSize(cols, rows);
                REQUIRE(matsClose(src, srcCopy, 0.0));

                auto raw = frame.getFrame();
                REQUIRE(raw.type() == CV_8UC1);
                REQUIRE(raw.cols == cols);
                REQUIRE(raw.rows == rows * 3 / 2);

                auto restored = frame.getCvFrame();
                REQUIRE(restored.type() == CV_8UC3);
                REQUIRE(matsClose(restored, src.clone(), 3.0));
            }
        }
    }

    SECTION("Grayscale and raw depths") {
        auto gray8Cont = makePattern(CV_8UC1, true);
        auto gray8Roi = makePattern(CV_8UC1, false);
        auto gray16Roi = makePattern(CV_16UC1, false);
        auto gray32Roi = makePattern(CV_32SC1, false);
        auto gray32F = makePattern(CV_32FC1, true);

        for(const auto& src : {gray8Cont, gray8Roi}) {
            ImgFrame frame;
            auto srcCopy = src.clone();
            frame.setCvFrame(src, ImgFrame::Type::RAW8).setSourceSize(cols, rows);
            REQUIRE(matsClose(src, srcCopy, 0.1));
            auto restored = frame.getCvFrame();
            REQUIRE(restored.type() == CV_8UC1);
            REQUIRE(restored.rows == rows);
            REQUIRE(restored.cols == cols);
            REQUIRE(matsClose(restored, src.clone(), 0.1));
        }

        ImgFrame raw16;
        auto gray16Copy = gray16Roi.clone();
        raw16.setCvFrame(gray16Roi, ImgFrame::Type::RAW16).setSourceSize(cols, rows);
        REQUIRE(matsClose(gray16Roi, gray16Copy, 0.0));
        auto raw16Frame = raw16.getFrame();
        REQUIRE(raw16Frame.type() == CV_16UC1);
        REQUIRE(matsClose(raw16.getCvFrame(), gray16Roi.clone(), 0.0));

        ImgFrame grayF16;
        cv::Mat cvGray16f;
        gray32F.convertTo(cvGray16f, CV_16FC1);
        auto gray16fCopy = cvGray16f.clone();
        grayF16.setCvFrame(cvGray16f, ImgFrame::Type::GRAYF16).setSourceSize(cols, rows);
        REQUIRE(matsClose(cvGray16f, gray16fCopy, 0.1));
        auto grayF16Frame = grayF16.getFrame();
        REQUIRE(grayF16Frame.type() == CV_16FC1);
        REQUIRE(matsClose(grayF16.getCvFrame(), cvGray16f, 1e-3));

        ImgFrame raw32;
        auto gray32Copy = gray32Roi.clone();
        raw32.setCvFrame(gray32Roi, ImgFrame::Type::RAW32).setSourceSize(cols, rows);
        REQUIRE(matsClose(gray32Roi, gray32Copy, 0.1));
        auto raw32Frame = raw32.getFrame();
        REQUIRE_FALSE(raw32Frame.empty());
    }

    ImgFrame empty;
    empty.setType(ImgFrame::Type::BGR888i);
    REQUIRE_THROWS_AS(empty.getFrame(), std::runtime_error);
}
#endif
