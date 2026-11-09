#include <catch2/catch_all.hpp>
#include <limits>

#include "../image_manip_fp16_test_utils.hpp"

using namespace dai::impl;
using fp16test::Type;

TEST_CASE("ImageManip FP16 specs and allocation agree", "[imagemanip][fp16]") {
    for(auto type : fp16test::types) {
        CAPTURE(type);
        auto frame = fp16test::makeFrame(type, true);
        const auto specs = getSrcFrameSpecs(frame->fb);
        CHECK(specs.p1Offset == 16);
        CHECK(specs.p1Stride == frame->fb.stride);
        CHECK(getFrameSize(type, specs) == frame->getData().size());
        auto tight = getDstFrameSpecs(23, 17, type);
        CHECK(getAlignedOutputFrameSize(type, 23, 17) == getFrameSize(type, tight));
        int planes = 0;
        float bpp = 0;
        REQUIRE(getFrameTypeInfo(type, planes, bpp));
        CHECK(getFrameSize(type, tight) == 23 * 17 * planes * bpp);
    }
}

TEST_CASE("Host ImageManip warps signed fractional FP16 planes", "[imagemanip][fp16]") {
    const bool padded = GENERATE(false, true);
    enum class Transform { Identity, Affine, Perspective };
    const auto transform = GENERATE(Transform::Identity, Transform::Affine, Transform::Perspective);
    for(auto type : fp16test::types) {
        CAPTURE(type, padded, transform);
        auto input = fp16test::makeFrame(type, padded);
        auto output = fp16test::makeFrame(type);
        const auto srcSpecs = getSrcFrameSpecs(input->fb);
        const auto dstSpecs = getSrcFrameSpecs(output->fb);
        const std::array<std::array<float, 3>, 3> matrix = {{{1, 0, transform == Transform::Identity ? 0.f : 1.25f},
                                                             {0, 1, transform == Transform::Identity ? 0.f : -0.5f},
                                                             {transform == Transform::Perspective ? 0.03125f : 0.f, 0, 1}}};
        WarpH warp;
        warp.build(srcSpecs, dstSpecs, type, matrix, {});
        warp.setBackgroundColor(3, 7, 11);
        warp.apply(std::make_shared<_ImageManipMemory>(input->getData()), std::make_shared<_ImageManipMemory>(output->getData()));
        fp16test::checkWarp(*output, *input, matrix, {warp.backgroundColor[0], warp.backgroundColor[1], warp.backgroundColor[2]});
    }
}

TEST_CASE("ImageManip FP16 layout conversion preserves floating values", "[imagemanip][fp16]") {
    for(auto from : fp16test::types) {
        for(auto to : fp16test::types) {
            CAPTURE(from, to);
            auto input = fp16test::makeFrame(from, true);
            auto output = fp16test::makeFrame(to, true);
            auto restored = fp16test::makeFrame(from);
            auto src = std::make_shared<_ImageManipMemory>(input->getData());
            auto dst = std::make_shared<_ImageManipMemory>(output->getData());
            ColorChangeH cc;
            cc.build(getSrcFrameSpecs(input->fb), getSrcFrameSpecs(output->fb), from, to);
            cc.apply(src, dst);
            if(from != Type::GRAYF16 && to == Type::GRAYF16) {
                auto inputPlanes = fp16test::planes(*input);
                cv::Mat rgb;
                if(inputPlanes.size() == 3)
                    cv::merge(inputPlanes, rgb);
                else
                    rgb = inputPlanes[0];
                rgb.convertTo(rgb, CV_32F);
                const bool bgr = from == Type::BGRF16F16F16p || from == Type::BGRF16F16F16i;
                cv::Mat expected, actual;
                cv::cvtColor(rgb, expected, bgr ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
                fp16test::planes(*output)[0].convertTo(actual, CV_32F);
                CHECK(cv::norm(actual, expected, cv::NORM_INF) < 0.016);
            } else {
                cc.build(getSrcFrameSpecs(output->fb), getSrcFrameSpecs(restored->fb), to, from);
                cc.apply(dst, std::make_shared<_ImageManipMemory>(restored->getData()));
                auto expected = fp16test::planes(*input);
                auto actual = fp16test::planes(*restored);
                for(size_t p = 0; p < expected.size(); ++p) {
                    for(int y = 0; y < expected[p].rows; ++y)
                        CHECK(std::memcmp(expected[p].ptr(y), actual[p].ptr(y), expected[p].cols * expected[p].elemSize()) == 0);
                }
            }
        }
    }
}

TEST_CASE("ImageManip FP16 converts integer samples without normalization", "[imagemanip][fp16]") {
    const auto integerType = GENERATE(Type::GRAY8, Type::RGB888i, Type::BGR888i, Type::RGB888p, Type::BGR888p, Type::NV12, Type::YUV420p);
    for(auto halfType : fp16test::types) {
        CAPTURE(integerType, halfType);
        dai::ImgFrame input;
        cv::Mat image(18, 24, CV_8UC3, cv::Scalar(12, 89, 203));
        input.setCvFrame(image, integerType);
        const auto inputSpecs = getSrcFrameSpecs(input.fb);
        const auto halfSpecs = getDstFrameSpecs(24, 18, halfType);
        auto half = std::make_shared<_ImageManipMemory>(getFrameSize(halfType, halfSpecs));
        auto source = std::make_shared<_ImageManipMemory>(input.getData());
        auto actual = std::make_shared<_ImageManipMemory>(getFrameSize(integerType, inputSpecs));
        ColorChangeH cc;
        cc.build(inputSpecs, halfSpecs, integerType, halfType);
        cc.apply(source, half);
        cc.build(halfSpecs, inputSpecs, halfType, integerType);
        cc.apply(half, actual);
        // A color-to-gray conversion is intentionally lossy; all other paths
        // preserve constant integer samples to YUV coefficient rounding.
        if(halfType != Type::GRAYF16 || integerType == Type::GRAY8) {
            for(size_t i = 0; i < actual->getOffsetSize(); ++i) CHECK(std::abs(int(actual->getOffsetData()[i]) - int(source->getOffsetData()[i])) <= 2);
        }
    }
}

TEST_CASE("Host ImageManip FP16 undistortion interpolates floating samples", "[imagemanip][fp16]") {
    const std::array<float, 9> intrinsics = {20, 0, 11, 0, 20, 8, 0, 0, 1};
    const std::vector<float> distortion = {0.12f, 0.02f, 0, 0, 0};
    cv::Mat map1, map2;
    cv::Mat camera(3, 3, CV_32F, const_cast<float*>(intrinsics.data()));
    cv::initUndistortRectifyMap(camera, distortion, cv::Mat(), camera, cv::Size(23, 17), CV_16SC2, map1, map2);
    for(auto type : fp16test::types) {
        CAPTURE(type);
        auto input = fp16test::makeFrame(type, true);
        auto output = fp16test::makeFrame(type);
        UndistortOpenCvImpl undistort(nullptr);
        REQUIRE(undistort.build(intrinsics, intrinsics, distortion, type, 23, 17, 23, 17) == Undistort::BuildStatus::ONE_SHOT);
        auto srcPlanes = fp16test::planes(*input);
        auto dstPlanes = fp16test::planes(*output);
        for(size_t p = 0; p < srcPlanes.size(); ++p) {
            auto src = std::make_shared<_ImageManipMemory>(dai::span<uint8_t>(srcPlanes[p].data, srcPlanes[p].data + srcPlanes[p].step * srcPlanes[p].rows));
            auto dst = std::make_shared<_ImageManipMemory>(dai::span<uint8_t>(dstPlanes[p].data, dstPlanes[p].data + dstPlanes[p].step * dstPlanes[p].rows));
            undistort.undistort({23, 17, srcPlanes[p].step, 2, static_cast<uint32_t>(srcPlanes[p].channels()), src},
                                {23, 17, dstPlanes[p].step, 2, static_cast<uint32_t>(dstPlanes[p].channels()), dst},
                                p);
            cv::Mat source, actual, expected;
            srcPlanes[p].convertTo(source, CV_32F);
            dstPlanes[p].convertTo(actual, CV_32F);
            cv::remap(source, expected, map1, map2, cv::INTER_LINEAR);
            CHECK(cv::norm(actual, expected, cv::NORM_INF) < 0.016);
        }
    }
}

TEST_CASE("Host ImageManip updates FP16 strides and clears conversion state", "[imagemanip][fp16]") {
    ImageManipOperations<_ImageManipMemory, ColorChangeH, WarpH> manip(dai::ImageManipProperties{});
    dai::ImageManipConfig config;
    config.addTransformPerspective({1, 0, 1.25f, 0, 1, -0.5f, 0, 0, 1}).setOutputSize(23, 17);
    config.base.setOutputCenter(false);
    std::vector<uint8_t> previous;
    for(bool padded : {false, true}) {
        auto input = fp16test::makeFrame(Type::RGBF16F16F16p, padded);
        manip.build(config.base, Type::BGRF16F16F16i, getSrcFrameSpecs(input->fb), input->getType());
        auto output = std::make_shared<_ImageManipMemory>(manip.getOutputSize());
        CHECK(manip.getOutputSize() == 23 * 17 * 6);
        CHECK(manip.getOutputPlaneSize() == 23 * 17 * 6);
        REQUIRE(manip.apply(std::make_shared<_ImageManipMemory>(input->getData()), output));
        const auto data = output->getOffsetData();
        if(padded)
            CHECK(std::equal(data.begin(), data.end(), previous.begin(), previous.end()));
        else
            previous.assign(data.begin(), data.end());
    }
    auto input = fp16test::makeFrame(Type::RGBF16F16F16p);
    dai::ImageManipConfig identity;
    manip.build(identity.base, input->getType(), getSrcFrameSpecs(input->fb), input->getType());
    CHECK(manip.getOutputSize() == 0);
}

TEST_CASE("Host ImageManip sequences FP16 undistortion and warping", "[imagemanip][fp16]") {
    const bool oneShot = GENERATE(false, true);
    const std::array<float, 9> intrinsics = {20, 0, 11, 0, 20, 8, 0, 0, 1};
    const std::array<float, 9> adjusted = {20, 0, 12.25f, 0, 20, 7.5f, 0, 0, 1};
    const std::vector<float> distortion = {0.12f, 0.02f, 0, 0, 0};
    const std::array<std::array<float, 3>, 3> matrix = {{{1, 0, 1.25f}, {0, 1, -0.5f}, {0, 0, 1}}};
    cv::Mat camera(3, 3, CV_32F, const_cast<float*>(intrinsics.data()));
    cv::Mat newCamera(3, 3, CV_32F, const_cast<float*>((oneShot ? adjusted : intrinsics).data()));
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(camera, distortion, cv::Mat(), newCamera, cv::Size(23, 17), CV_16SC2, map1, map2);
    for(auto type : fp16test::types) {
        CAPTURE(type, oneShot);
        auto input = fp16test::makeFrame(type, true);
        auto output = fp16test::makeFrame(type);
        WarpH warp;
        warp.build(getSrcFrameSpecs(input->fb), getSrcFrameSpecs(output->fb), type, matrix, {});
        warp.buildUndistort(true, intrinsics, oneShot ? adjusted : std::array<float, 9>{}, distortion, {}, type, 23, 17, 23, 17);
        REQUIRE(warp.enableUndistort);
        CHECK(warp.undistortOneShot == oneShot);
        warp.apply(std::make_shared<_ImageManipMemory>(input->getData()), std::make_shared<_ImageManipMemory>(output->getData()));
        auto srcPlanes = fp16test::planes(*input);
        auto dstPlanes = fp16test::planes(*output);
        for(size_t p = 0; p < srcPlanes.size(); ++p) {
            cv::Mat source, expected, actual;
            srcPlanes[p].convertTo(source, CV_32F);
            dstPlanes[p].convertTo(actual, CV_32F);
            cv::remap(source, expected, map1, map2, cv::INTER_LINEAR);
            if(!oneShot) {
                // Match the intermediate FP16 image between the two operations.
                expected.convertTo(expected, CV_16F);
                expected.convertTo(expected, CV_32F);
                cv::warpPerspective(expected, expected, cv::Mat(3, 3, CV_32F, const_cast<float*>(matrix[0].data())), expected.size(), cv::INTER_LINEAR);
            }
            CHECK(cv::norm(actual, expected, cv::NORM_INF) <= 0.032);
        }
    }
}

TEST_CASE("ImageManip FP16 conversion preserves magnitudes and saturates integers", "[imagemanip][fp16]") {
    const auto integerSpecs = getDstFrameSpecs(9, 2, Type::GRAY8);
    const auto halfSpecs = getDstFrameSpecs(9, 2, Type::GRAYF16);
    auto integers = std::make_shared<_ImageManipMemory>(getAlignedOutputFrameSize(Type::GRAY8, 9, 2));
    auto halves = std::make_shared<_ImageManipMemory>(getFrameSize(Type::GRAYF16, halfSpecs));
    cv::Mat integerImage(2, 9, CV_8UC1, integers->getOffsetData().data(), integerSpecs.p1Stride);
    cv::Mat halfImage(2, 9, CV_16FC1, halves->getOffsetData().data(), halfSpecs.p1Stride);
    integerImage.setTo(128);
    ColorChangeH cc;
    cc.build(integerSpecs, halfSpecs, Type::GRAY8, Type::GRAYF16);
    cc.apply(integers, halves);
    for(int y = 0; y < 2; ++y) {
        for(int x = 0; x < 9; ++x) CHECK(static_cast<float>(halfImage.at<cv::float16_t>(y, x)) == 128.f);
    }
    const std::array<float, 9> input = {
        -3.f, -0.25f, 0.25f, 1.5f, 254.5f, 255.75f, 300.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()};
    const std::array<uint8_t, 9> expected = {0, 0, 0, 2, 255, 255, 255, 255, 0};
    for(int y = 0; y < 2; ++y) {
        for(int x = 0; x < 9; ++x) halfImage.at<cv::float16_t>(y, x) = cv::float16_t(input[x]);
    }
    cc.build(halfSpecs, integerSpecs, Type::GRAYF16, Type::GRAY8);
    cc.apply(halves, integers);
    for(int y = 0; y < 2; ++y) {
        for(int x = 0; x < 9; ++x) CHECK(integerImage.at<uint8_t>(y, x) == expected[x]);
    }
}

TEST_CASE("ImageManip planar specs infer offsets relative to the first plane", "[imagemanip][fp16]") {
    const auto type = GENERATE(Type::RGB888p, Type::BGR888p, Type::RGBF16F16F16p, Type::BGRF16F16F16p);
    dai::ImgFrame::Specs source{};
    source.type = type;
    source.width = 23;
    source.height = 17;
    source.stride = 64;
    source.p1Offset = 16;
    const auto inferred = getSrcFrameSpecs(source);
    CHECK(inferred.p1Offset == 16);
    CHECK(inferred.p2Offset == 16 + 64 * 17);
    CHECK(inferred.p3Offset == 16 + 2 * 64 * 17);
    source.p2Offset = 1200;
    source.p3Offset = 0;
    const auto explicitSecond = getSrcFrameSpecs(source);
    CHECK(explicitSecond.p2Offset == 1200);
    CHECK(explicitSecond.p3Offset == 1200 + 64 * 17);
}
