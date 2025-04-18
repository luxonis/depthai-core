#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

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
    // Use multiple manips to properly check DS for different types
    auto manipDeviceGRAY8 = p.create<dai::node::ImageManipV2>();
    manipDeviceGRAY8->setRunOnHost(false);
    manipDeviceGRAY8->inputConfig.setReusePreviousMessage(false);
    manipDeviceGRAY8->inputConfig.setWaitForMessage(true);
    manipDeviceGRAY8->setMaxOutputFrameSize(3000000);
    manipDeviceGRAY8->setNumFramesPool(1);
    auto manipDeviceNV12 = p.create<dai::node::ImageManipV2>();
    manipDeviceNV12->setRunOnHost(false);
    manipDeviceNV12->inputConfig.setReusePreviousMessage(false);
    manipDeviceNV12->inputConfig.setWaitForMessage(true);
    manipDeviceNV12->setMaxOutputFrameSize(3000000);
    manipDeviceNV12->setNumFramesPool(1);
    auto manipDeviceRGB888i = p.create<dai::node::ImageManipV2>();
    manipDeviceRGB888i->setRunOnHost(false);
    manipDeviceRGB888i->inputConfig.setReusePreviousMessage(false);
    manipDeviceRGB888i->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888i->setMaxOutputFrameSize(3000000);
    manipDeviceRGB888i->setNumFramesPool(1);
    auto manipDeviceRGB888p = p.create<dai::node::ImageManipV2>();
    manipDeviceRGB888p->setRunOnHost(false);
    manipDeviceRGB888p->inputConfig.setReusePreviousMessage(false);
    manipDeviceRGB888p->inputConfig.setWaitForMessage(true);
    manipDeviceRGB888p->setMaxOutputFrameSize(3000000);
    manipDeviceRGB888p->setNumFramesPool(1);
    auto manipHost = p.create<dai::node::ImageManipV2>();
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
        REQUIRE(diff < 10.);
    };

    auto doConfig = [&](std::shared_ptr<dai::ImageManipConfigV2> cfg) {
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
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(300, 300, dai::ImageManipConfigV2::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        // Full scale up
        std::cout << "FSU" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::STRETCH);
        doConfig(cfg);
    }
    {
        std::cout << "C" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        // Lenna should be 512x512
        cfg->addCrop(100, 50, 400, 400);
        doConfig(cfg);
    }
    {
        // Letterbox scale up
        std::cout << "LSU" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 800, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Letterbox scale down
        std::cout << "LSD" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(400, 300, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
        doConfig(cfg);
    }
    {
        // Setting background
        std::cout << "BS" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::NONE);
        cfg->setBackgroundColor(0, 0, 0);
        doConfig(cfg);
    }
    {
        // Setting background
        std::cout << "BM" << std::endl;
        auto cfg = std::make_shared<dai::ImageManipConfigV2>();
        cfg->setOutputSize(1000, 1000, dai::ImageManipConfigV2::ResizeMode::NONE);
        cfg->setBackgroundColor(200, 0, 0);
        doConfig(cfg);
    }

    p.stop();
}
