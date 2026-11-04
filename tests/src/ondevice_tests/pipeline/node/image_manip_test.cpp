#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <utility>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/properties/ImageManipProperties.hpp"

namespace {

std::vector<cv::Mat> fractionalCropPlanes(dai::ImgFrame& frame) {
    if(frame.getType() != dai::ImgFrame::Type::NV12) return {frame.getFrame()};

    auto* data = frame.getData().data();
    return {cv::Mat(frame.getHeight(), frame.getWidth(), CV_8UC1, data, frame.getStride()),
            cv::Mat(frame.getHeight() / 2, frame.getWidth() / 2, CV_8UC2, data + frame.getPlaneStride(0), frame.getStride())};
}

cv::Mat createCropTestImage(int width, int height) {
    cv::Mat image(height, width, CV_8UC3);
    cv::RNG rng(7061);
    rng.fill(image, cv::RNG::UNIFORM, 0, 256);
    // Smooth texture limits sensitivity to subpixel interpolation differences,
    // while asymmetric shapes make rotation, translation and scale errors visible.
    cv::GaussianBlur(image, image, cv::Size(7, 7), 1.5);
    cv::rectangle(image, cv::Rect(35, 45, 125, 80), cv::Scalar(30, 220, 70), cv::FILLED);
    cv::rectangle(image, cv::Rect(280, 180, 95, 55), cv::Scalar(230, 60, 180), 5);
    cv::circle(image, cv::Point(330, 90), 38, cv::Scalar(180, 240, 20), cv::FILLED);
    cv::line(image, cv::Point(60, 270), cv::Point(420, 130), cv::Scalar(250, 30, 120), 5);
    return image;
}

struct ManipCropCase {
    cv::RotatedRect rectangle;
    bool axisAligned;
};

std::vector<ManipCropCase> createManipCropCases() {
    const std::array<cv::Size2f, 4> sizes = {{{64, 64}, {80, 128}, {192, 112}, {128.5f, 80.25f}}};
    const std::array<cv::Point2f, 3> centers = {{{256, 160}, {140.25f, 115.75f}, {365.5f, 205.25f}}};
    std::vector<ManipCropCase> cases;
    cases.reserve(sizes.size() * centers.size() * 26);
    for(const auto& size : sizes) {
        for(const auto& center : centers) {
            // All corners stay in bounds even at 45 degrees; source-border
            // extrapolation is independent of crop geometry and interpolation.
            for(int angle = -180; angle <= 180; angle += 15) {
                cases.push_back({cv::RotatedRect(center, size, static_cast<float>(angle)), false});
            }
            // Exercise addCrop as well as the zero-angle rotated-rectangle API.
            cases.push_back({cv::RotatedRect(center, size, 0), true});
        }
    }
    return cases;
}

cv::Mat referenceManipCrop(const cv::Mat& image, const cv::RotatedRect& rectangle, dai::Platform platform) {
    auto transform = cv::getRotationMatrix2D(rectangle.center, rectangle.angle, 1.0);
    transform.at<double>(0, 2) += rectangle.size.width / 2.0 - rectangle.center.x;
    transform.at<double>(1, 2) += rectangle.size.height / 2.0 - rectangle.center.y;
    if(platform == dai::Platform::RVC2) {
        // RVC2 samples its warp mesh endpoints at the first and last output
        // pixels, whereas OpenCV uses an extent ending one pixel past the last.
        transform.row(0) *= (static_cast<int>(rectangle.size.width) - 1.0) / static_cast<int>(rectangle.size.width);
        transform.row(1) *= (static_cast<int>(rectangle.size.height) - 1.0) / static_cast<int>(rectangle.size.height);
    }
    cv::Mat expected;
    // Crops retain unit scale and truncate fractional raster dimensions.
    cv::warpAffine(image, expected, transform, cv::Size(static_cast<int>(rectangle.size.width), static_cast<int>(rectangle.size.height)), cv::INTER_LINEAR);
    return expected;
}

void checkManipCrop(const cv::Mat& actual, const cv::Mat& expected) {
    REQUIRE(actual.size() == expected.size());
    REQUIRE(actual.type() == expected.type());
    cv::Mat difference;
    cv::absdiff(actual, expected, difference);
    const auto channels = difference.reshape(1);
    const double meanError = cv::mean(channels)[0];
    const double outlierFraction = static_cast<double>(cv::countNonZero(channels > 16)) / channels.total();
    CAPTURE(meanError, outlierFraction);
    // Compare corresponding pixels, allowing rounding and interpolation-table
    // approximations. A few high-contrast edge pixels may differ more, but a
    // displaced or incorrectly rotated crop still produces substantial errors.
    CHECK(meanError <= 2.0);
    CHECK(outlierFraction <= 0.01);
}

}  // namespace

TEST_CASE("ImageManip crops match OpenCV across rotations sizes and offsets", "[imagemanip][crop][opencv]") {
    const bool normalizedCoords = GENERATE(false, true);
    const auto type = GENERATE(dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::BGR888i);
    CAPTURE(normalizedCoords, type);
    constexpr int inputWidth = 512, inputHeight = 320;
    auto image = createCropTestImage(inputWidth, inputHeight);
    if(type == dai::ImgFrame::Type::GRAY8) cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    dai::Pipeline pipeline;
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->setMaxOutputFrameSize(inputWidth * inputHeight * 3);
    manip->inputConfig.setWaitForMessage(true);
    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();
    pipeline.start();

    int sequence = 0;
    for(const auto& crop : createManipCropCases()) {
        const auto& rect = crop.rectangle;
        CAPTURE(crop.axisAligned, rect.center.x, rect.center.y, rect.size.width, rect.size.height, rect.angle);
        const float xScale = normalizedCoords ? 1.f / inputWidth : 1.f;
        const float yScale = normalizedCoords ? 1.f / inputHeight : 1.f;
        auto config = std::make_shared<dai::ImageManipConfig>();
        if(crop.axisAligned) {
            config->addCrop(dai::Rect((rect.center.x - rect.size.width / 2) * xScale,
                                      (rect.center.y - rect.size.height / 2) * yScale,
                                      rect.size.width * xScale,
                                      rect.size.height * yScale,
                                      normalizedCoords),
                            normalizedCoords);
        } else {
            config->addCropRotatedRect(dai::RotatedRect(dai::Point2f(rect.center.x * xScale, rect.center.y * yScale, normalizedCoords),
                                                        dai::Size2f(rect.size.width * xScale, rect.size.height * yScale, normalizedCoords),
                                                        rect.angle),
                                       normalizedCoords);
        }
        config->setFrameType(type);
        auto input = std::make_shared<dai::ImgFrame>();
        input->setCvFrame(image, type);
        input->setSequenceNum(sequence);
        configQueue->send(config);
        inputQueue->send(input);
        bool timedOut = false;
        auto output = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output != nullptr);
        REQUIRE(output->getType() == type);
        REQUIRE(output->getSequenceNum() == sequence++);
        checkManipCrop(output->getCvFrame(), referenceManipCrop(image, rect, pipeline.getDefaultDevice()->getPlatform()));
    }
}

TEST_CASE("ImageManip rotated crops retain boundary pixels", "[imagemanip][crop-boundary][rvc4]") {
    const bool normalizedCoords = GENERATE(false, true);
    const auto type = GENERATE(dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::BGR888i, dai::ImgFrame::Type::NV12);
    CAPTURE(normalizedCoords, type);
    constexpr int inputWidth = 512, inputHeight = 320;
    const std::array<cv::RotatedRect, 5> rectangles = {{{{256, 160}, {64, 64}, 0},
                                                        {{256.25f, 160.75f}, {64.5f, 48.5f}, 0},
                                                        {{32, 32}, {64, 64}, 0},
                                                        {{480, 288}, {64, 64}, 0},
                                                        {{479.75f, 295.75f}, {64.5f, 48.5f}, 0}}};
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("RVC4 warp source-boundary regression");
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->setBackend(dai::ImageManipProperties::Backend::CPU);
    manip->setMaxOutputFrameSize(inputWidth * inputHeight * 3);
    manip->inputConfig.setWaitForMessage(true);
    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();
    auto input = std::make_shared<dai::ImgFrame>();
    input->setCvFrame(createCropTestImage(inputWidth, inputHeight), type);
    const auto sourcePlanes = fractionalCropPlanes(*input);
    pipeline.start();

    for(auto rectangle : rectangles) {
        for(const float angle : {-180.f, -90.f, 0.f, 15.f, 90.f, 180.f}) {
            rectangle.angle = angle;
            CAPTURE(rectangle.center.x, rectangle.center.y, rectangle.size.width, rectangle.size.height, angle);
            auto config = std::make_shared<dai::ImageManipConfig>();
            auto crop = dai::RotatedRect(dai::Point2f(rectangle.center.x, rectangle.center.y), dai::Size2f(rectangle.size.width, rectangle.size.height), angle);
            if(normalizedCoords) crop = crop.normalize(inputWidth, inputHeight);
            config->addCropRotatedRect(crop, normalizedCoords);
            config->setFrameType(type);
            configQueue->send(config);
            inputQueue->send(input);
            bool timedOut = false;
            auto output = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(output != nullptr);
            REQUIRE(output->getWidth() == static_cast<int>(rectangle.size.width));
            REQUIRE(output->getHeight() == static_cast<int>(rectangle.size.height));
            REQUIRE(output->getType() == type);
            const auto outputPlanes = fractionalCropPlanes(*output);
            REQUIRE(outputPlanes.size() == sourcePlanes.size());
            for(std::size_t plane = 0; plane < sourcePlanes.size(); ++plane) {
                CAPTURE(plane);
                const float subsampling = plane == 0 ? 1.f : 2.f;
                auto transform = cv::getRotationMatrix2D(rectangle.center, angle, 1.0);
                transform.at<double>(0, 2) = (transform.at<double>(0, 2) + rectangle.size.width / 2.0 - rectangle.center.x) / subsampling;
                transform.at<double>(1, 2) = (transform.at<double>(1, 2) + rectangle.size.height / 2.0 - rectangle.center.y) / subsampling;
                cv::Mat expected;
                const auto background = type == dai::ImgFrame::Type::NV12 ? (plane == 0 ? cv::Scalar(16) : cv::Scalar(128, 128)) : cv::Scalar();
                cv::warpAffine(sourcePlanes[plane], expected, transform, outputPlanes[plane].size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, background);
                // Include every border pixel and both NV12 planes. Real image
                // boundaries still use black padding, exactly as in OpenCV.
                CHECK(cv::norm(outputPlanes[plane], expected, cv::NORM_INF) <= 4.0);
            }
        }
    }
}

TEST_CASE("ImageManip fractional crops at unit scale preserve pixels", "[imagemanip][fractional-crop][rvc4]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("RVC4 crop fast-path regression");

    constexpr int inputWidth = 512, inputHeight = 288;
    constexpr int outputWidth = 128, outputHeight = 256;
    using ResizeMode = dai::ImageManipConfig::ResizeMode;
    struct CropCase {
        float x, y, width, height;
        ResizeMode mode;
    };
    // Keep scale 1 to exercise crop classification. NONE allows both dimensions
    // to be fractional while centering the crop in the fixed-size output.
    const std::array<CropCase, 12> cases = {{{74.f, 26.f, 128.f, 256.f, ResizeMode::STRETCH},
                                             {73.9375f, 26.75f, 128.f, 256.f, ResizeMode::STRETCH},
                                             {73.9375f, 26.75f, 128.f, 256.f, ResizeMode::CENTER_CROP},
                                             {74.f, 27.f, 128.f, 257.f, ResizeMode::CENTER_CROP},
                                             {74.f, 26.f, 128.f, 258.f, ResizeMode::CENTER_CROP},
                                             {75.f, 27.f, 128.f, 256.f, ResizeMode::STRETCH},
                                             {74.f, 26.f, 129.5f, 256.f, ResizeMode::CENTER_CROP},
                                             {73.9375f, 26.75f, 129.5f, 256.f, ResizeMode::CENTER_CROP},
                                             {74.f, 26.f, 128.f, 257.5f, ResizeMode::CENTER_CROP},
                                             {73.9375f, 26.75f, 128.f, 257.5f, ResizeMode::CENTER_CROP},
                                             {74.f, 26.f, 129.5f, 257.5f, ResizeMode::NONE},
                                             {73.9375f, 26.75f, 129.5f, 257.5f, ResizeMode::NONE}}};
    const std::array<dai::ImgFrame::Type, 3> types = {dai::ImgFrame::Type::BGR888i, dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::NV12};

    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->setBackend(dai::ImageManipProperties::Backend::CPU);
    manip->setMaxOutputFrameSize(outputWidth * outputHeight * 3);
    manip->inputConfig.setWaitForMessage(true);
    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();
    pipeline.start();

    cv::RNG rng(7061);
    int sequence = 0;
    for(int iteration = 0; iteration < 3; ++iteration) {
        for(const auto type : types) {
            cv::Mat image(inputHeight, inputWidth, type == dai::ImgFrame::Type::GRAY8 ? CV_8UC1 : CV_8UC3);
            rng.fill(image, cv::RNG::UNIFORM, 0, 256);
            for(const auto& crop : cases) {
                CAPTURE(iteration, type, crop.x, crop.y, crop.width, crop.height, crop.mode);
                auto input = std::make_shared<dai::ImgFrame>();
                input->setCvFrame(image, type);
                input->setSequenceNum(sequence);
                auto config = std::make_shared<dai::ImageManipConfig>();
                config->addCropRotatedRect(
                    dai::RotatedRect(dai::Point2f(crop.x + crop.width / 2.f, crop.y + crop.height / 2.f), dai::Size2f(crop.width, crop.height), 0), false);
                config->setOutputSize(outputWidth, outputHeight, crop.mode);
                config->setFrameType(type);
                configQueue->send(config);
                inputQueue->send(input);

                bool timedOut = false;
                auto output = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
                REQUIRE_FALSE(timedOut);
                REQUIRE(output != nullptr);
                REQUIRE(output->getWidth() == outputWidth);
                REQUIRE(output->getHeight() == outputHeight);
                REQUIRE(output->getType() == type);
                REQUIRE(output->getSequenceNum() == sequence++);

                const auto sourcePlanes = fractionalCropPlanes(*input);
                const auto outputPlanes = fractionalCropPlanes(*output);
                REQUIRE(sourcePlanes.size() == outputPlanes.size());
                for(std::size_t plane = 0; plane < sourcePlanes.size(); ++plane) {
                    CAPTURE(plane);
                    const float ssF = plane == 0 ? 1.f : 2.f;
                    // Crop dimensions become integer raster sizes before
                    // centering. All cases retain scale 1 and split any excess
                    // width and height equally on either side of the output.
                    const cv::Matx23f transform(1,
                                                0,
                                                -(crop.x + (std::floor(crop.width) - outputWidth) / 2.f) / ssF,
                                                0,
                                                1,
                                                -(crop.y + (std::floor(crop.height) - outputHeight) / 2.f) / ssF);
                    cv::Mat expected;
                    cv::warpAffine(sourcePlanes[plane], expected, transform, outputPlanes[plane].size(), cv::INTER_LINEAR);
                    // Check every pixel: an oversized row copy corrupts pixels
                    // even when an adjacent allocation conceals the overrun.
                    REQUIRE(cv::norm(outputPlanes[plane], expected, cv::NORM_INF) <= 1.0);
                }
            }
        }
    }
}

void testManipBasic(bool runSyncOnHost) {
    // Create pipeline
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManip>();
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    manip->initialConfig->addCrop(100, 200, 400, 400);
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::NV12);

    auto manipQueue = manip->out.createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        auto inFrame = manipQueue->get<dai::ImgFrame>();
        REQUIRE(inFrame != nullptr);
        REQUIRE(inFrame->getWidth() == 400);
        REQUIRE(inFrame->getHeight() == 400);
        REQUIRE(inFrame->getType() == dai::ImgFrame::Type::NV12);
    }
}

void testManipDynamic(bool runSyncOnHost, bool reuiseImage) {
    // Create pipeline
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManip>();
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    auto manipQueue = manip->out.createOutputQueue();
    manip->inputConfig.setReusePreviousMessage(false);  // Control the rate with the config input
    auto configQueue = manip->inputConfig.createInputQueue();
    p.start();
    int cropX = 100;
    int cropY = 200;
    int cropW = 400;
    int cropH = 400;
    for(int i = 0; i < 10; i++) {
        cropX = cropX + 10;
        cropY = cropY + 10;
        cropW = cropW + 10;
        cropH = cropH + 10;
        auto manipConfig = std::make_shared<dai::ImageManipConfig>();
        manipConfig->addCrop(cropX, cropY, cropW, cropH);
        manipConfig->setFrameType(dai::ImgFrame::Type::NV12);
        manipConfig->setReusePreviousImage(reuiseImage);
        configQueue->send(manipConfig);
        auto inFrame = manipQueue->get<dai::ImgFrame>();
        REQUIRE(inFrame != nullptr);
        REQUIRE(inFrame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(inFrame->getWidth() == cropW);
        REQUIRE(inFrame->getHeight() == cropH);
    }
}

TEST_CASE("Sync node runs on device") {
    testManipBasic(false);
}

TEST_CASE("Sync node runs on host") {
    testManipBasic(true);
}

TEST_CASE("Dynamic config sync node runs on device, reuse") {
    testManipDynamic(false, true);
}

TEST_CASE("Dynamic config sync node runs on host, reuse") {
    testManipDynamic(true, true);
}

TEST_CASE("Dynamic config sync node runs on device, no reuse") {
    testManipDynamic(false, false);
}

TEST_CASE("Dynamic config sync node runs on host, no reuse") {
    testManipDynamic(true, false);
}

TEST_CASE("Test ImageManip with u16 frames") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    constexpr int inputWidth = 640, inputHeight = 480;
    constexpr int outputWidth = 320, outputHeight = 320;
    constexpr size_t N = 20;

    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>();
    manip->initialConfig->setOutputSize(outputWidth, outputHeight);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    p.start();

    for(size_t i = 0; i < N; ++i) {
        auto inFrame = std::make_shared<dai::ImgFrame>();
        inFrame->setData(std::vector<uint8_t>(inputWidth * inputHeight * 2));
        inFrame->setWidth(inputWidth);
        inFrame->setHeight(inputHeight);
        inFrame->setStride(inputWidth * 2);
        inFrame->setType(dai::ImgFrame::Type::RAW16);

        inputQueue->send(inFrame);

        // Retrieve the resized frame
        auto outFrame = outputQueue->get<dai::ImgFrame>();

        REQUIRE(outFrame->getWidth() == outputWidth);
        REQUIRE(outFrame->getHeight() == outputHeight);
    }
}

TEST_CASE("ImageManip rebuild on cfg change") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camOut = cam->requestOutput({1280, 720});
    auto manip = p.create<dai::node::ImageManip>();
    manip->inputConfig.setWaitForMessage(true);

    auto camQueue = camOut->createOutputQueue(1, false);
    auto manipQueue = manip->out.createOutputQueue(1, false);
    auto ifQueue = manip->inputImage.createInputQueue();
    auto icQueue = manip->inputConfig.createInputQueue();
    p.start();
    dai::ImageManipConfig cfg;
    cfg.setOutputSize(400, 200);
    ifQueue->send(camQueue->get<dai::ImgFrame>());
    icQueue->send(std::make_shared<dai::ImageManipConfig>(cfg));
    auto imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 400);
    REQUIRE(imgFrame->getHeight() == 200);
    cfg.setOutputSize(200, 400);
    ifQueue->send(camQueue->get<dai::ImgFrame>());
    icQueue->send(std::make_shared<dai::ImageManipConfig>(cfg));
    imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 200);
    REQUIRE(imgFrame->getHeight() == 400);
    p.stop();
}

TEST_CASE("Multiple image manips") {
    dai::Pipeline p;
    auto inputImg = cv::imread(LENNA_PATH);
    cv::resize(inputImg, inputImg, cv::Size(1024, 512));
    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setCvFrame(inputImg, dai::ImgFrame::Type::NV12);

    uint32_t resizeWidth = 256;
    uint32_t resizeHeight = 256;
    int numNodes = 10;

    auto config = std::make_shared<dai::ImageManipConfig>();
    config->setOutputSize(resizeWidth, resizeHeight);

    std::vector<std::shared_ptr<dai::node::ImageManip>> manipNodes;
    std::vector<std::shared_ptr<dai::InputQueue>> manipQins;
    std::vector<std::shared_ptr<dai::MessageQueue>> manipQouts;

    for(int i = 0; i < numNodes; ++i) {
        auto manip = p.create<dai::node::ImageManip>();
        manip->setBackend(dai::ImageManipProperties::Backend::CPU);
        manip->setPerformanceMode(dai::ImageManipProperties::PerformanceMode::BALANCED);
        manip->setMaxOutputFrameSize(2097152);
        manip->initialConfig = config;
        auto manipQin = manip->inputImage.createInputQueue();
        auto manipQout = manip->out.createOutputQueue();

        manipNodes.push_back(manip);
        manipQins.push_back(manipQin);
        manipQouts.push_back(manipQout);
    }

    p.start();

    for(int i = 0; i < 100; i++) {
        for(int j = 0; j < numNodes; ++j) {
            manipQins[j]->send(inputFrame);
        }

        std::vector<std::shared_ptr<dai::ImgFrame>> frames;
        for(int j = 0; j < numNodes; ++j) {
            auto frame = manipQouts[j]->get<dai::ImgFrame>();
            frames.push_back(frame);
            REQUIRE(frame->getWidth() == resizeWidth);
            REQUIRE(frame->getHeight() == resizeHeight);
        }
    }

    p.stop();
}

bool equal(const cv::Mat& a, const cv::Mat& b) {
    cv::Scalar s = cv::sum(a != b);
    for(int i = 0; i < a.dims; i++) {
        if(a.size[i] != b.size[i] || s[i] > 0) return false;
    }
    return true;
}
double compareHistograms(const cv::Mat& img1, const cv::Mat& img2) {
    // Validate dimensions
    if(img1.empty() || img2.empty()) {
        std::cerr << "Empty image(s) provided." << std::endl;
        return -1.0;
    }

    // Ensure both images have the same number of channels
    if(img1.channels() != img2.channels()) {
        std::cerr << "Image channel mismatch." << std::endl;
        return -1.0;
    }

    cv::Mat hist1, hist2;

    if(img1.channels() == 1) {
        // Grayscale histogram
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};

        cv::calcHist(&img1, 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange, true, false);
        cv::calcHist(&img2, 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange, true, false);
    } else {
        // Convert to HSV for color histogram
        cv::Mat hsv1, hsv2;
        cv::cvtColor(img1, hsv1, cv::COLOR_BGR2HSV);
        cv::cvtColor(img2, hsv2, cv::COLOR_BGR2HSV);

        int h_bins = 50, s_bins = 60;
        int histSize[] = {h_bins, s_bins};
        float h_range[] = {0, 180};
        float s_range[] = {0, 256};
        const float* ranges[] = {h_range, s_range};
        int channels[] = {0, 1};

        cv::calcHist(&hsv1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);
        cv::calcHist(&hsv2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);
    }

    // Normalize both histograms
    cv::normalize(hist1, hist1, 0, 1, cv::NORM_MINMAX);
    cv::normalize(hist2, hist2, 0, 1, cv::NORM_MINMAX);

    // Compare histograms (correlation: 1 = identical)
    double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_CORREL);
    return similarity;
}

cv::Mat createFourPointTransformInputImage(int width, int height) {
    cv::Mat image(height, width, CV_8UC1);
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            image.at<uint8_t>(y, x) = static_cast<uint8_t>((x * 3 + y * 5) % 256);
        }
    }

    cv::rectangle(image, cv::Rect(20, 20, 80, 60), cv::Scalar(255), cv::FILLED);
    cv::circle(image, cv::Point(width - 50, 50), 30, cv::Scalar(32), cv::FILLED);
    cv::line(image, cv::Point(0, height - 1), cv::Point(width - 1, 0), cv::Scalar(180), 3);
    cv::putText(image, "4PT", cv::Point(60, height - 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(90), 2);

    return image;
}

std::array<dai::Point2f, 4> normalizeFourPointCoords(const std::array<dai::Point2f, 4>& points, int width, int height) {
    std::array<dai::Point2f, 4> normalized = points;
    for(auto& point : normalized) {
        point.x /= static_cast<float>(width);
        point.y /= static_cast<float>(height);
    }
    return normalized;
}

void testManipFourPointTransform(bool normalizedCoords) {
    constexpr int width = 256;
    constexpr int height = 192;
    const std::array<dai::Point2f, 4> srcPoints = {
        dai::Point2f(40.0f, 30.0f), dai::Point2f(216.0f, 30.0f), dai::Point2f(216.0f, 162.0f), dai::Point2f(40.0f, 162.0f)};
    const std::array<dai::Point2f, 4> dstPoints = {
        dai::Point2f(0.0f, 0.0f), dai::Point2f(width, 0.0f), dai::Point2f(width, height), dai::Point2f(0.0f, height)};
    struct Marker {
        cv::Point source;
        cv::Point destination;
    };
    // These positions map exactly under the configured source and destination rectangles.
    // Keeping them in separate quadrants verifies both axes of the transform across the image.
    const std::array<Marker, 4> markers = {
        Marker{{62, 52}, {32, 32}},
        Marker{{194, 52}, {224, 32}},
        Marker{{62, 140}, {32, 160}},
        Marker{{194, 140}, {224, 160}},
    };

    cv::Mat inputImage = cv::Mat::zeros(height, width, CV_8UC1);
    for(const auto& marker : markers) {
        cv::circle(inputImage, marker.source, 8, cv::Scalar(255), cv::FILLED);
    }

    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setCvFrame(inputImage, dai::ImgFrame::Type::GRAY8);

    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>();
    manip->inputConfig.setWaitForMessage(true);
    manip->setMaxOutputFrameSize(width * height);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    p.start();

    auto runManip = [&](const std::shared_ptr<dai::ImageManipConfig>& cfg) {
        configQueue->send(cfg);
        inputQueue->send(inputFrame);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == width);
        REQUIRE(outFrame->getHeight() == height);
        return outFrame->getCvFrame().clone();
    };

    auto fourPointCfg = std::make_shared<dai::ImageManipConfig>();
    if(normalizedCoords) {
        fourPointCfg->addTransformFourPoints(normalizeFourPointCoords(srcPoints, width, height), normalizeFourPointCoords(dstPoints, width, height), true);
    } else {
        fourPointCfg->addTransformFourPoints(srcPoints, dstPoints, false);
    }
    fourPointCfg->setOutputSize(width, height);
    fourPointCfg->setFrameType(dai::ImgFrame::Type::GRAY8);

    const auto outImage = runManip(fourPointCfg);
    constexpr int patchRadius = 8;
    for(const auto& marker : markers) {
        const cv::Rect patchRect{marker.destination.x - patchRadius, marker.destination.y - patchRadius, 2 * patchRadius + 1, 2 * patchRadius + 1};
        cv::Mat matchingPixels = outImage(patchRect) > 127;
        REQUIRE(cv::countNonZero(matchingPixels) >= 100);
    }

    p.stop();
}

void runManipTests(dai::ImgFrame::Type type, bool undistort, bool useCoeffs = true) {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto manip = p.create<dai::node::ImageManip>()->build();
    manip->setMaxOutputFrameSize(6750208);
    manip->inputConfig.setWaitForMessage(true);

    auto config = std::make_shared<dai::ImageManipConfig>();

    auto camOutQ = camera->requestOutput({1024, 512})->createOutputQueue();
    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    auto inputImg = cv::imread(LENNA_PATH);

    p.start();

    auto getFrames = [&](std::shared_ptr<dai::ImageManipConfig> _cfg, uint32_t outWidth, uint32_t outHeight) {
        auto frame = camOutQ->get<dai::ImgFrame>();
        if(!useCoeffs) frame->transformation.setDistortionCoefficients({});
        cv::Mat inputImg2;
        cv::resize(inputImg, inputImg2, cv::Size(frame->getWidth(), frame->getHeight()));
        frame->setCvFrame(inputImg2, type);
        inputQueue->send(frame);
        configQueue->send(_cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == outWidth);
        REQUIRE(outFrame->getHeight() == outHeight);
        return std::make_pair(frame, outFrame);
    };
    auto getFrame = [&](std::shared_ptr<dai::ImageManipConfig> _cfg, uint32_t outWidth, uint32_t outHeight) {
        return getFrames(_cfg, outWidth, outHeight).second;
    };

    // Scale up
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1400, 700);
        auto outFrame1 = getFrame(cfg, 1400, 700);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 1400, 700);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale up crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
        auto outFrame1 = getFrame(cfg, 1500, 1500);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 1500, 1500);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale up letterbox
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        auto outFrame1 = getFrame(cfg, 1500, 1500);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 1500, 1500);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale up letterbox bg
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        cfg->setBackgroundColor(100, 0, 0);
        auto outFrame1 = getFrame(cfg, 1500, 1500);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 1500, 1500);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale down
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400);
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale down crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale down letterbox
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale down letterbox bg
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        cfg->setBackgroundColor(100, 0, 0);
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCrop(100, 200, 600, 400);
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Affine
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCropRotatedRect(dai::RotatedRect(dai::Point2f(350, 250), dai::Size2f(600, 400), 20));
        auto outFrame1 = getFrame(cfg, 600, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 600, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Scale down small
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCrop(100, 100, 199, 199);
        cfg->setOutputSize(100, 100);
        auto outFrame1 = getFrame(cfg, 100, 100);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 100, 100);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    // Rotate
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addRotateDeg(10);
        cfg->setOutputSize(400, 400);
        auto outFrame1 = getFrame(cfg, 400, 400);
        if(undistort) {
            cfg->setUndistort(true);
            auto outFrame2 = getFrame(cfg, 400, 400);
            if(undistort && useCoeffs) REQUIRE(!equal(outFrame1->getCvFrame(), outFrame2->getCvFrame()));
        }
    }

    if(undistort && useCoeffs) {
        // Undistort only
        {
            auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
            cfg->setUndistort(true);
            auto frames = getFrames(cfg, 1024, 512);
            REQUIRE(compareHistograms(frames.first->getCvFrame(), frames.second->getCvFrame()) > 0.6);
            REQUIRE(!equal(frames.first->getCvFrame(), frames.second->getCvFrame()));
        }
    }

    p.stop();
}

TEST_CASE("ImageManip NV12") {
    runManipTests(dai::ImgFrame::Type::NV12, false);
}

TEST_CASE("ImageManip four point transform") {
    testManipFourPointTransform(false);
}

TEST_CASE("ImageManip four point transform normalized coordinates") {
    testManipFourPointTransform(true);
}

TEST_CASE("ImageManip crop and four point transform with stretch") {
    constexpr int inputWidth = 400;
    constexpr int inputHeight = 300;
    constexpr int outputWidth = 45;
    constexpr int outputHeight = 36;

    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setCvFrame(createFourPointTransformInputImage(inputWidth, inputHeight), dai::ImgFrame::Type::GRAY8);

    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>();
    manip->inputConfig.setWaitForMessage(true);
    manip->setMaxOutputFrameSize(inputWidth * inputHeight);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    p.start();

    auto cfg = std::make_shared<dai::ImageManipConfig>();
    cfg->addCropRotatedRect(dai::RotatedRect(dai::Point2f(0.45f, 0.04f, true), dai::Size2f(0.12f, 0.12f, true), 0.0f), true);
    cfg->addTransformFourPoints({dai::Point2f(0.44f, 0.83f), dai::Point2f(0.15f, 0.36f), dai::Point2f(0.54f, 0.17f), dai::Point2f(0.82f, 0.42f)},
                                {dai::Point2f(0.0f, 0.0f), dai::Point2f(1.0f, 0.0f), dai::Point2f(1.0f, 1.0f), dai::Point2f(0.0f, 1.0f)},
                                true);
    cfg->setOutputSize(outputWidth, outputHeight, dai::ImageManipConfig::ResizeMode::STRETCH);
    cfg->setFrameType(dai::ImgFrame::Type::GRAY8);

    configQueue->send(cfg);
    inputQueue->send(inputFrame);

    bool timedOut = false;
    auto outFrame = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(outFrame != nullptr);
    REQUIRE(outFrame->getWidth() == outputWidth);
    REQUIRE(outFrame->getHeight() == outputHeight);
}

TEST_CASE("ImageManip GRAY8") {
    runManipTests(dai::ImgFrame::Type::GRAY8, false);
}

TEST_CASE("ImageManip RGB888i") {
    runManipTests(dai::ImgFrame::Type::RGB888i, false);
}

TEST_CASE("ImageManip NV12 undistort no coefficients") {
    runManipTests(dai::ImgFrame::Type::NV12, true, false);
}

TEST_CASE("ImageManip NV12 undistort") {
    runManipTests(dai::ImgFrame::Type::NV12, true);
}

TEST_CASE("ImageManip GRAY8 undistort") {
    runManipTests(dai::ImgFrame::Type::GRAY8, true);
}

TEST_CASE("ImageManip RGB888i undistort") {
    runManipTests(dai::ImgFrame::Type::RGB888i, true);
}
