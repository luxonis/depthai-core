#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>

#include "depthai/depthai.hpp"

namespace {

struct GeneratorOutput {
    std::shared_ptr<dai::ImageManipConfig> config;
    std::shared_ptr<dai::ImgFrame> image;
};

GeneratorOutput runGenerator(const std::shared_ptr<dai::Buffer>& detections, const std::shared_ptr<dai::ImgFrame>& image) {
    dai::Pipeline pipeline(false);
    auto generator = pipeline.create<dai::node::CropConfigGenerator>();
    generator->setRunOnHost(true);

    auto detectionsQueue = generator->inputDetections.createInputQueue();
    auto imageQueue = generator->inputImage.createInputQueue();
    auto configQueue = generator->outConfig.createOutputQueue();
    auto outputImageQueue = generator->outImage.createOutputQueue();

    pipeline.start();
    detectionsQueue->send(detections);
    imageQueue->send(image);

    bool configTimedOut = false;
    bool imageTimedOut = false;
    auto config = configQueue->get<dai::ImageManipConfig>(std::chrono::seconds(1), configTimedOut);
    auto outputImage = outputImageQueue->get<dai::ImgFrame>(std::chrono::seconds(1), imageTimedOut);
    REQUIRE_FALSE(configTimedOut);
    REQUIRE_FALSE(imageTimedOut);
    REQUIRE(config);
    REQUIRE(outputImage);
    return {config, outputImage};
}

void requireCrop(const dai::ImageManipConfig& config, float centerX, float centerY, float width, float height, float angle) {
    const auto& operations = config.base.getOperations();
    REQUIRE(operations.size() == 2);

    const auto& translate = std::get<dai::Translate>(operations.at(0).op);
    CHECK(translate.offsetX == Catch::Approx(-centerX + width / 2.0F));
    CHECK(translate.offsetY == Catch::Approx(-centerY + height / 2.0F));
    CHECK(translate.normalized);

    const auto& crop = std::get<dai::CropRotated>(operations.at(1).op);
    CHECK(crop.width == Catch::Approx(width));
    CHECK(crop.height == Catch::Approx(height));
    CHECK(crop.angle == Catch::Approx(angle));
    CHECK(crop.normalized);
}

std::shared_ptr<dai::ImgFrame> makeImage() {
    auto image = std::make_shared<dai::ImgFrame>();
    image->setSize(640, 400);
    image->setSequenceNum(42);
    return image;
}

}  // namespace

TEST_CASE("CropConfigGenerator generates paired crops from ImgDetections", "[CropConfigGenerator]") {
    auto detections = std::make_shared<dai::ImgDetections>();
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.4F, 0.3F, true), dai::Size2f(0.2F, 0.1F, true), 15.0F));
    auto image = makeImage();

    const auto output = runGenerator(detections, image);

    requireCrop(*output.config, 0.4F, 0.3F, 0.2F, 0.1F, 15.0F);
    CHECK(output.config->getSequenceNum() == image->getSequenceNum());
    CHECK(output.image == image);
}

TEST_CASE("CropConfigGenerator supports SpatialImgDetections", "[CropConfigGenerator]") {
    auto detections = std::make_shared<dai::SpatialImgDetections>();
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.55F, 0.45F, true), dai::Size2f(0.3F, 0.25F, true), -10.0F),
                                        dai::Point3f(0.0F, 0.0F, 1000.0F));

    const auto output = runGenerator(detections, makeImage());
    requireCrop(*output.config, 0.55F, 0.45F, 0.3F, 0.25F, -10.0F);
}

TEST_CASE("CropConfigGenerator supports Tracklets", "[CropConfigGenerator]") {
    auto tracklets = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.1F, 0.2F, 0.4F, 0.3F, true);
    tracklets->tracklets.push_back(tracklet);

    const auto output = runGenerator(tracklets, makeImage());
    requireCrop(*output.config, 0.3F, 0.35F, 0.4F, 0.3F, 0.0F);
}

TEST_CASE("CropConfigGenerator configs are applied to their paired images", "[CropConfigGenerator][ImageManip]") {
    dai::Pipeline pipeline(false);
    auto generator = pipeline.create<dai::node::CropConfigGenerator>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    generator->setRunOnHost(true);
    manip->setRunOnHost(true);
    manip->inputConfig.setWaitForMessage(true);

    generator->outConfig.link(manip->inputConfig);
    generator->outImage.link(manip->inputImage);

    auto detectionsQueue = generator->inputDetections.createInputQueue();
    auto imageQueue = generator->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    auto detections = std::make_shared<dai::ImgDetections>();
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.2F, 0.25F, true), dai::Size2f(0.2F, 0.3F, true), 0.0F));
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.6F, 0.6F, true), dai::Size2f(0.4F, 0.5F, true), 0.0F));

    auto image = std::make_shared<dai::ImgFrame>();
    image->setSourceSize(10, 10);
    image->setSize(10, 10);
    image->setType(dai::ImgFrame::Type::RAW8);
    image->setData(std::vector<std::uint8_t>(100, 1));

    pipeline.start();
    detectionsQueue->send(detections);
    imageQueue->send(image);

    bool firstTimedOut = false;
    bool secondTimedOut = false;
    const auto first = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(1), firstTimedOut);
    const auto second = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(1), secondTimedOut);
    REQUIRE_FALSE(firstTimedOut);
    REQUIRE_FALSE(secondTimedOut);
    REQUIRE(first);
    REQUIRE(second);
    CHECK(first->getWidth() == 2);
    CHECK(first->getHeight() == 3);
    CHECK(second->getWidth() == 4);
    CHECK(second->getHeight() == 5);
}
