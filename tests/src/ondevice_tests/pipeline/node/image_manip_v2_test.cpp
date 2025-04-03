#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

void testManipBasic(bool runSyncOnHost) {
    // Create pipeline
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManipV2>();
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    manip->initialConfig.addCrop(100, 200, 400, 400);
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);

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
    auto manip = p.create<dai::node::ImageManipV2>();
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
        auto manipConfig = std::make_shared<dai::ImageManipConfigV2>();
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

TEST_CASE("Test ImageManipV2 with u16 frames") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    constexpr int inputWidth = 640, inputHeight = 480;
    constexpr int outputWidth = 320, outputHeight = 320;
    constexpr size_t N = 20;

    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManipV2>();
    manip->initialConfig.setOutputSize(outputWidth, outputHeight);

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

TEST_CASE("ImageManipV2 rebuild on cfg change") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camOut = cam->requestOutput({1280, 720});
    auto manip = p.create<dai::node::ImageManipV2>();
    manip->setRunOnHost(true);
    manip->inputConfig.setWaitForMessage(true);

    auto camQueue = camOut->createOutputQueue(1, false);
    auto manipQueue = manip->out.createOutputQueue(1, false);
    auto ifQueue = manip->inputImage.createInputQueue();
    auto icQueue = manip->inputConfig.createInputQueue();
    p.start();
    dai::ImageManipConfigV2 cfg;
    cfg.setOutputSize(400, 200);
    ifQueue->send(camQueue->get<dai::ImgFrame>());
    icQueue->send(std::make_shared<dai::ImageManipConfigV2>(cfg));
    auto imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 400);
    REQUIRE(imgFrame->getHeight() == 200);
    cfg.setOutputSize(200, 400);
    ifQueue->send(camQueue->get<dai::ImgFrame>());
    icQueue->send(std::make_shared<dai::ImageManipConfigV2>(cfg));
    imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 200);
    REQUIRE(imgFrame->getHeight() == 400);
    p.stop();
}

// Function to calculate the normalized difference between two images
double calculateImageDifference(const cv::Mat& img1, const cv::Mat& img2) {
    // Ensure the images have the same size and type
    if(img1.size() != img2.size() || img1.type() != img2.type()) {
        std::cerr << "Error: Images must have the same size and type!" << std::endl;
        return -1.0;
    }

    // Compute absolute difference between images
    cv::Mat diff;
    cv::absdiff(img1, img2, diff);

    // Sum all pixel differences
    double sumDiff = cv::sum(diff)[0];  // Sum of all pixels (for grayscale)

    // If the image has multiple channels (e.g., RGB), sum across all channels
    if(img1.channels() > 1) {
        cv::Scalar sumValues = cv::sum(diff);
        sumDiff = sumValues[0] + sumValues[1] + sumValues[2];  // Sum across all channels
    }

    // Normalize by the total number of pixels
    double numPixels = img1.total() * img1.channels();
    double normalizedDifference = sumDiff / numPixels;

    return normalizedDifference;
}

TEST_CASE("Host and Device impl comparison") {
    dai::Pipeline p;
    auto manipDevice = p.create<dai::node::ImageManipV2>();
    manipDevice->setRunOnHost(false);
    manipDevice->inputConfig.setReusePreviousMessage(false);
    manipDevice->inputConfig.setWaitForMessage(true);
    manipDevice->setMaxOutputFrameSize(3000000);
    auto manipHost = p.create<dai::node::ImageManipV2>();
    manipHost->setRunOnHost(true);
    manipHost->inputConfig.setReusePreviousMessage(false);
    manipHost->inputConfig.setWaitForMessage(true);
    manipHost->setMaxOutputFrameSize(3000000);

    manipDevice->inputConfig.setWaitForMessage(true);
    manipHost->inputConfig.setWaitForMessage(true);

    auto inputImg = cv::imread(LENNA_PATH);
    auto inputFrameGRAY8 = std::make_shared<dai::ImgFrame>();
    inputFrameGRAY8->setCvFrame(inputImg, dai::ImgFrame::Type::GRAY8);
    auto inputFrameNV12 = std::make_shared<dai::ImgFrame>();
    inputFrameNV12->setCvFrame(inputImg, dai::ImgFrame::Type::NV12);
    auto inputFrameRGB888i = std::make_shared<dai::ImgFrame>();
    inputFrameRGB888i->setCvFrame(inputImg, dai::ImgFrame::Type::RGB888i);
    auto inputFrameRGB888p = std::make_shared<dai::ImgFrame>();
    inputFrameRGB888p->setCvFrame(inputImg, dai::ImgFrame::Type::RGB888p);

    auto manipDeviceConfigQ = manipDevice->inputConfig.createInputQueue();
    auto manipHostConfigQ = manipHost->inputConfig.createInputQueue();
    auto manipDeviceImgQ = manipDevice->inputImage.createInputQueue();
    auto manipHostImgQ = manipHost->inputImage.createInputQueue();
    auto manipDeviceOutQ = manipDevice->out.createOutputQueue(1);
    auto manipHostOutQ = manipHost->out.createOutputQueue(1);

    auto doImgConfig = [&](std::shared_ptr<dai::ImgFrame> img, std::shared_ptr<dai::ImageManipConfigV2> cfg) {
        manipDeviceConfigQ->send(cfg);
        manipHostConfigQ->send(cfg);
        manipDeviceImgQ->send(img);
        manipHostImgQ->send(img);

        auto hostFrame = manipHostOutQ->get<dai::ImgFrame>();
        auto deviceFrame = manipDeviceOutQ->get<dai::ImgFrame>();

        auto hostImg = hostFrame->getCvFrame();
        auto deviceImg = deviceFrame->getCvFrame();

        REQUIRE(hostImg.size() == deviceImg.size());
        REQUIRE(hostImg.type() == deviceImg.type());
        REQUIRE(hostImg.channels() == deviceImg.channels());
        REQUIRE(hostImg.total() == deviceImg.total());
        REQUIRE(hostImg.rows == deviceImg.rows);
        REQUIRE(hostImg.cols == deviceImg.cols);
        auto diff = calculateImageDifference(hostImg, deviceImg);
        REQUIRE(diff < 2.);
    };
    auto doConfig = [&](std::shared_ptr<dai::ImageManipConfigV2> cfg) {
        doImgConfig(inputFrameGRAY8, cfg);
        doImgConfig(inputFrameNV12, cfg);
        doImgConfig(inputFrameRGB888i, cfg);
        doImgConfig(inputFrameRGB888p, cfg);
    };

    p.start();

    // Check operations: crop, scale up, scale down, background
    {
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        // Lenna should be 512x512
        cfg->addCrop(100, 50, 400, 400);
        doConfig(cfg);
    }
    {
        // Full scale up
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        // Full scale down
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(300, 300, dai::ImageManipConfigV2::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        // Letterbox scale up
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1600, 1000, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Letterbox scale down
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(400, 300, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Setting background
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::NONE);
        cfg->setBackgroundColor(0, 0, 0);
        doConfig(cfg);
    }
    {
        // Setting background
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::NONE);
        cfg->setBackgroundColor(200, 0, 0);
        doConfig(cfg);
    }


    p.stop();
}
