#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <utility>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/properties/ImageManipProperties.hpp"

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
