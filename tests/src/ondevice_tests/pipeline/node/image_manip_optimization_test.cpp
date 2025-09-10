#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/ImageManipProperties.hpp"

double calculateImageDifference(const cv::Mat& img1, const cv::Mat& img2, int blockSize = 8) {
    if(img1.empty() || img2.empty()) {
        std::cerr << "One of the input images is empty!" << std::endl;
        return 0.0;
    }

    int numChannels = img1.channels();

    // Resize images to be the same size and divisible by blockSize
    cv::Size commonSize = img1.size();
    commonSize.width -= commonSize.width % blockSize;
    commonSize.height -= commonSize.height % blockSize;
    cv::Mat resized1, resized2;
    cv::resize(img1, resized1, commonSize);
    cv::resize(img2, resized2, commonSize);

    int blocksX = commonSize.width / blockSize;
    int blocksY = commonSize.height / blockSize;

    double totalDiff = 0.0;
    int count = 0;

    for(int y = 0; y < blocksY; ++y) {
        for(int x = 0; x < blocksX; ++x) {
            cv::Rect roi(x * blockSize, y * blockSize, blockSize, blockSize);
            cv::Mat block1 = resized1(roi);
            cv::Mat block2 = resized2(roi);

            cv::Mat hist1, hist2;
            if(numChannels == 3) {
                // Compute color histograms
                int histSize[] = {8, 8, 8};  // BGR histogram
                float range[] = {0, 256};
                const float* ranges[] = {range, range, range};
                int channels[] = {0, 1, 2};

                cv::calcHist(&block1, 1, channels, cv::Mat(), hist1, 3, histSize, ranges, true, false);
                cv::calcHist(&block2, 1, channels, cv::Mat(), hist2, 3, histSize, ranges, true, false);
            } else if(numChannels == 1) {
                // Compute grayscale histogram
                int histSize = 256;
                float range[] = {0, 256};
                const float* ranges[] = {range};

                cv::calcHist(&block1, 1, 0, cv::Mat(), hist1, 1, &histSize, ranges, true, false);
                cv::calcHist(&block2, 1, 0, cv::Mat(), hist2, 1, &histSize, ranges, true, false);
            } else {
                std::cerr << "Unsupported number of channels: " << numChannels << std::endl;
                return 0.0;
            }

            cv::normalize(hist1, hist1);
            cv::normalize(hist2, hist2);

            cv::Mat histDiff = hist1 - hist2;
            double diff = cv::norm(histDiff, cv::NORM_L2);
            totalDiff += diff;
            ++count;
        }
    }

    return count > 0 ? totalDiff / count : 0.0;
}

TEST_CASE("Host and Device impl comparison") {
    dai::Pipeline p;
    // Use multiple manips to properly check DS for different types
    auto manipDeviceGRAY8 = p.create<dai::node::ImageManip>();
    manipDeviceGRAY8->setRunOnHost(false);
    manipDeviceGRAY8->inputConfig.setReusePreviousMessage(false);
    manipDeviceGRAY8->inputConfig.setWaitForMessage(true);
    manipDeviceGRAY8->setMaxOutputFrameSize(3000000);
    manipDeviceGRAY8->setNumFramesPool(1);
    auto manipDeviceNV12 = p.create<dai::node::ImageManip>();
    manipDeviceNV12->setRunOnHost(false);
    manipDeviceNV12->inputConfig.setReusePreviousMessage(false);
    manipDeviceNV12->inputConfig.setWaitForMessage(true);
    manipDeviceNV12->setMaxOutputFrameSize(3000000);
    manipDeviceNV12->setNumFramesPool(1);
    auto manipDeviceRGB888i = p.create<dai::node::ImageManip>();
    manipDeviceRGB888i->setRunOnHost(false);
    manipDeviceRGB888i->inputConfig.setReusePreviousMessage(false);
    manipDeviceRGB888i->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888i->setMaxOutputFrameSize(3000000);
    manipDeviceRGB888i->setNumFramesPool(1);
    auto manipDeviceRGB888p = p.create<dai::node::ImageManip>();
    manipDeviceRGB888p->setRunOnHost(false);
    manipDeviceRGB888p->inputConfig.setReusePreviousMessage(false);
    manipDeviceRGB888p->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888p->setMaxOutputFrameSize(3000000);
    manipDeviceRGB888p->setNumFramesPool(1);
    auto manipHost = p.create<dai::node::ImageManip>();
    manipHost->setRunOnHost(true);
    manipHost->inputConfig.setReusePreviousMessage(false);
    manipHost->inputConfig.setWaitForMessage(true);
    manipHost->setMaxOutputFrameSize(3000000);
    manipHost->setNumFramesPool(1);

    manipDeviceGRAY8->inputConfig.setWaitForMessage(true);
    manipDeviceNV12->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888i->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888p->inputConfig.setWaitForMessage(true);
    manipHost->inputConfig.setWaitForMessage(true);

    auto inputImg = cv::imread(LENNA_PATH);
    cv::resize(inputImg, inputImg, cv::Size(512, 512));
    auto inputFrameGRAY8 = std::make_shared<dai::ImgFrame>();
    inputFrameGRAY8->setCvFrame(inputImg, dai::ImgFrame::Type::GRAY8);
    auto inputFrameNV12 = std::make_shared<dai::ImgFrame>();
    inputFrameNV12->setCvFrame(inputImg, dai::ImgFrame::Type::NV12);
    auto inputFrameRGB888i = std::make_shared<dai::ImgFrame>();
    inputFrameRGB888i->setCvFrame(inputImg, dai::ImgFrame::Type::RGB888i);
    auto inputFrameRGB888p = std::make_shared<dai::ImgFrame>();
    inputFrameRGB888p->setCvFrame(inputImg, dai::ImgFrame::Type::RGB888p);

    auto manipDeviceConfigGRAY8Q = manipDeviceGRAY8->inputConfig.createInputQueue();
    auto manipDeviceImgGRAY8Q = manipDeviceGRAY8->inputImage.createInputQueue();
    auto manipDeviceOutGRAY8Q = manipDeviceGRAY8->out.createOutputQueue(10);
    auto manipDeviceConfigNV12Q = manipDeviceNV12->inputConfig.createInputQueue();
    auto manipDeviceImgNV12Q = manipDeviceNV12->inputImage.createInputQueue();
    auto manipDeviceOutNV12Q = manipDeviceNV12->out.createOutputQueue(10);
    auto manipDeviceConfigRGB888iQ = manipDeviceRGB888i->inputConfig.createInputQueue();
    auto manipDeviceImgRGB888iQ = manipDeviceRGB888i->inputImage.createInputQueue();
    auto manipDeviceOutRGB888iQ = manipDeviceRGB888i->out.createOutputQueue(10);
    auto manipDeviceConfigRGB888pQ = manipDeviceRGB888p->inputConfig.createInputQueue();
    auto manipDeviceImgRGB888pQ = manipDeviceRGB888p->inputImage.createInputQueue();
    auto manipDeviceOutRGB888pQ = manipDeviceRGB888p->out.createOutputQueue(10);
    auto manipHostConfigQ = manipHost->inputConfig.createInputQueue();
    auto manipHostImgQ = manipHost->inputImage.createInputQueue();
    auto manipHostOutQ = manipHost->out.createOutputQueue(20);

    auto checkImgConfig = [&](cv::Mat hostImg, cv::Mat deviceImg) {
        REQUIRE(hostImg.size() == deviceImg.size());
        REQUIRE(hostImg.type() == deviceImg.type());
        REQUIRE(hostImg.channels() == deviceImg.channels());
        REQUIRE(hostImg.total() == deviceImg.total());
        REQUIRE(hostImg.rows == deviceImg.rows);
        REQUIRE(hostImg.cols == deviceImg.cols);
        auto diff = calculateImageDifference(hostImg, deviceImg);
        REQUIRE(diff < 0.8f);
    };

    auto doConfig = [&](std::shared_ptr<dai::ImageManipConfig> cfg) {
        {
            manipDeviceConfigGRAY8Q->send(cfg);
            manipHostConfigQ->send(cfg);
            manipDeviceImgGRAY8Q->send(inputFrameGRAY8);
            manipHostImgQ->send(inputFrameGRAY8);
        }
        {
            manipDeviceConfigNV12Q->send(cfg);
            manipHostConfigQ->send(cfg);
            manipDeviceImgNV12Q->send(inputFrameNV12);
            manipHostImgQ->send(inputFrameNV12);
        }
        {
            manipDeviceConfigRGB888iQ->send(cfg);
            manipHostConfigQ->send(cfg);
            manipDeviceImgRGB888iQ->send(inputFrameRGB888i);
            manipHostImgQ->send(inputFrameRGB888i);
        }
        {
            manipDeviceConfigRGB888pQ->send(cfg);
            manipHostConfigQ->send(cfg);
            manipDeviceImgRGB888pQ->send(inputFrameRGB888p);
            manipHostImgQ->send(inputFrameRGB888p);
        }
        {
            auto hostFrame = manipHostOutQ->get<dai::ImgFrame>();
            auto deviceFrame = manipDeviceOutGRAY8Q->get<dai::ImgFrame>();

            checkImgConfig(hostFrame->getCvFrame(), deviceFrame->getCvFrame());
        }
        {
            auto hostFrame = manipHostOutQ->get<dai::ImgFrame>();
            auto deviceFrame = manipDeviceOutNV12Q->get<dai::ImgFrame>();

            checkImgConfig(hostFrame->getCvFrame(), deviceFrame->getCvFrame());
        }
        {
            auto hostFrame = manipHostOutQ->get<dai::ImgFrame>();
            auto deviceFrame = manipDeviceOutRGB888iQ->get<dai::ImgFrame>();

            checkImgConfig(hostFrame->getCvFrame(), deviceFrame->getCvFrame());
        }
        {
            auto hostFrame = manipHostOutQ->get<dai::ImgFrame>();
            auto deviceFrame = manipDeviceOutRGB888pQ->get<dai::ImgFrame>();

            checkImgConfig(hostFrame->getCvFrame(), deviceFrame->getCvFrame());
        }
    };

    p.start();

    // Check operations: crop, scale up, scale down, background
    {
        // Full scale down
        std::cout << "FSD" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(300, 300, dai::ImageManipConfig::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        // Full scale up
        std::cout << "FSU" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfig::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        std::cout << "C" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        // Lenna should be 512x512
        cfg->addCrop(100, 50, 400, 400);
        doConfig(cfg);
    }
    {
        // Letterbox scale up
        std::cout << "LSU" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(1000, 800, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Letterbox scale down
        std::cout << "LSD" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(400, 300, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Setting background
        std::cout << "BS" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfig::ResizeMode::NONE);
        cfg->setBackgroundColor(0, 0, 0);
        doConfig(cfg);
    }
    {
        // Setting background
        std::cout << "BM" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfig>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfig::ResizeMode::NONE);
        cfg->setBackgroundColor(200, 0, 0);
        doConfig(cfg);
    }

    p.stop();
}

void runManipTests(dai::ImageManipProperties::Backend backend, dai::ImageManipProperties::PerformanceMode perfMode, dai::ImgFrame::Type type) {
    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>()->build();
    manip->setBackend(backend);
    manip->setPerformanceMode(perfMode);
    manip->setMaxOutputFrameSize(6750208);
    manip->inputConfig.setWaitForMessage(true);

    auto inputImg = cv::imread(LENNA_PATH);
    cv::resize(inputImg, inputImg, cv::Size(1024, 512));
    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setCvFrame(inputImg, type);

    auto config = std::make_shared<dai::ImageManipConfig>();
    config->setReusePreviousImage(true);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto configQueue = manip->inputConfig.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    p.start();
    inputQueue->send(inputFrame);

    // Scale up
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(2048, 1024);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 2048);
        REQUIRE(outFrame->getHeight() == 1024);
    }

    // Scale up crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 1500);
        REQUIRE(outFrame->getHeight() == 1500);
    }

    // Scale up letterbox
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 1500);
        REQUIRE(outFrame->getHeight() == 1500);
    }

    // Scale up letterbox bg
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(1500, 1500, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        cfg->setBackgroundColor(100, 0, 0);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 1500);
        REQUIRE(outFrame->getHeight() == 1500);
    }

    // Scale down
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Scale down crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Scale down letterbox
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Scale down letterbox bg
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->setOutputSize(600, 400, dai::ImageManipConfig::ResizeMode::LETTERBOX);
        cfg->setBackgroundColor(100, 0, 0);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Crop
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCrop(100, 200, 600, 400);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Affine
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCropRotatedRect(dai::RotatedRect(dai::Point2f(350, 250), dai::Size2f(600, 400), 20));
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 600);
        REQUIRE(outFrame->getHeight() == 400);
    }

    // Scale down small
    {
        auto cfg = std::make_shared<dai::ImageManipConfig>(*config);
        cfg->addCrop(100, 100, 199, 199);
        cfg->setOutputSize(100, 100);
        configQueue->send(cfg);
        auto outFrame = outputQueue->get<dai::ImgFrame>();
        REQUIRE(outFrame != nullptr);
        REQUIRE(outFrame->getWidth() == 100);
        REQUIRE(outFrame->getHeight() == 100);
    }

    p.stop();
}

TEST_CASE("ImageManip NV12 Low Power") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::LOW_POWER, dai::ImgFrame::Type::NV12);
}

TEST_CASE("ImageManip NV12 Balanced") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::NV12);
}

TEST_CASE("ImageManip NV12 Performance") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::PERFORMANCE, dai::ImgFrame::Type::NV12);
}

TEST_CASE("ImageManip NV12 HW") {
    runManipTests(dai::ImageManipProperties::Backend::HW, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::NV12);
}

TEST_CASE("ImageManip GRAY8 Low Power") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::LOW_POWER, dai::ImgFrame::Type::GRAY8);
}

TEST_CASE("ImageManip GRAY8 Balanced") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::GRAY8);
}

TEST_CASE("ImageManip GRAY8 Performance") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::PERFORMANCE, dai::ImgFrame::Type::GRAY8);
}

TEST_CASE("ImageManip GRAY8 HW") {
    runManipTests(dai::ImageManipProperties::Backend::HW, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::GRAY8);
}

TEST_CASE("ImageManip RGB888i Low Power") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::LOW_POWER, dai::ImgFrame::Type::RGB888i);
}

TEST_CASE("ImageManip RGB888i Balanced") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::RGB888i);
}

TEST_CASE("ImageManip RGB888i Performance") {
    runManipTests(dai::ImageManipProperties::Backend::CPU, dai::ImageManipProperties::PerformanceMode::PERFORMANCE, dai::ImgFrame::Type::RGB888i);
}

TEST_CASE("ImageManip RGB888i HW") {
    runManipTests(dai::ImageManipProperties::Backend::HW, dai::ImageManipProperties::PerformanceMode::BALANCED, dai::ImgFrame::Type::RGB888i);
}

TEST_CASE("Large frame resize test") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto* output = camera->requestOutput({7680, 4320});
    auto camQ = output->createOutputQueue();

    p.start();

    auto frame = camQ->get<dai::ImgFrame>();
    REQUIRE(frame != nullptr);
    REQUIRE(frame->getWidth() == 7680);
    REQUIRE(frame->getHeight() == 4320);

    p.stop();
}
