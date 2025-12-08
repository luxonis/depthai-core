#include <array>
#include <catch2/catch_all.hpp>
#include <cstdint>
#include <cstring>
#include <vector>

#include "depthai/depthai.hpp"

using Catch::Approx;

TEST_CASE("SpatialLocationCalculatorConfig tracks ROI updates") {
    dai::SpatialLocationCalculatorConfig config;
    REQUIRE(config.getConfigData().empty());

    dai::SpatialLocationCalculatorConfigData initial;
    initial.roi = dai::Rect(10.0F, 20.0F, 30.0F, 40.0F, false);
    initial.depthThresholds.lowerThreshold = 100;
    initial.depthThresholds.upperThreshold = 2000;
    initial.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MODE;
    initial.stepSize = 1;
    config.addROI(initial);

    auto rois = config.getConfigData();
    REQUIRE(rois.size() == 1);
    CHECK(rois.front().depthThresholds.lowerThreshold == initial.depthThresholds.lowerThreshold);
    CHECK(rois.front().depthThresholds.upperThreshold == initial.depthThresholds.upperThreshold);
    CHECK(rois.front().calculationAlgorithm == initial.calculationAlgorithm);
    CHECK(rois.front().stepSize == initial.stepSize);
    CHECK(rois.front().roi.x == Approx(initial.roi.x));
    CHECK(rois.front().roi.y == Approx(initial.roi.y));
    CHECK(rois.front().roi.width == Approx(initial.roi.width));
    CHECK(rois.front().roi.height == Approx(initial.roi.height));

    dai::SpatialLocationCalculatorConfigData overrideA;
    overrideA.roi = dai::Rect(0.05F, 0.05F, 0.25F, 0.25F, true);
    overrideA.depthThresholds.lowerThreshold = 300;
    overrideA.depthThresholds.upperThreshold = 4500;
    overrideA.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MAX;
    overrideA.stepSize = 6;

    dai::SpatialLocationCalculatorConfigData overrideB;
    overrideB.roi = dai::Rect(0.3F, 0.4F, 0.2F, 0.15F, true);
    overrideB.depthThresholds.lowerThreshold = 200;
    overrideB.depthThresholds.upperThreshold = 5200;
    overrideB.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
    overrideB.stepSize = 3;

    config.setROIs({overrideA, overrideB});
    rois = config.getConfigData();
    REQUIRE(rois.size() == 2);

    CHECK(rois[0].depthThresholds.lowerThreshold == overrideA.depthThresholds.lowerThreshold);
    CHECK(rois[0].depthThresholds.upperThreshold == overrideA.depthThresholds.upperThreshold);
    CHECK(rois[0].calculationAlgorithm == overrideA.calculationAlgorithm);
    CHECK(rois[0].stepSize == overrideA.stepSize);
    CHECK(rois[0].roi.x == Approx(overrideA.roi.x));
    CHECK(rois[0].roi.y == Approx(overrideA.roi.y));
    CHECK(rois[0].roi.width == Approx(overrideA.roi.width));
    CHECK(rois[0].roi.height == Approx(overrideA.roi.height));

    CHECK(rois[1].depthThresholds.lowerThreshold == overrideB.depthThresholds.lowerThreshold);
    CHECK(rois[1].depthThresholds.upperThreshold == overrideB.depthThresholds.upperThreshold);
    CHECK(rois[1].calculationAlgorithm == overrideB.calculationAlgorithm);
    CHECK(rois[1].stepSize == overrideB.stepSize);
    CHECK(rois[1].roi.x == Approx(overrideB.roi.x));
    CHECK(rois[1].roi.y == Approx(overrideB.roi.y));
    CHECK(rois[1].roi.width == Approx(overrideB.roi.width));
    CHECK(rois[1].roi.height == Approx(overrideB.roi.height));
}

TEST_CASE("SpatialLocationCalculator synthetic depth data test") {
    constexpr unsigned width = 640;
    constexpr unsigned height = 480;

    struct RoiSpec {
        dai::Rect roi;
        std::uint16_t depth;
        std::uint32_t widthPx;
        std::uint32_t heightPx;
    };

    const std::vector<RoiSpec> roiSpecs = {
        {dai::Rect(100.0F, 120.0F, 40.0F, 60.0F, false), static_cast<std::uint16_t>(400), 40, 60},
        {dai::Rect(320.0F, 200.0F, 64.0F, 72.0F, false), static_cast<std::uint16_t>(600), 64, 72},
        {dai::Rect(420.0F, 80.0F, 96.0F, 96.0F, false), static_cast<std::uint16_t>(800), 96, 96},
    };

    dai::Pipeline pipeline;
    auto spatial = pipeline.create<dai::node::SpatialLocationCalculator>();

    std::vector<dai::SpatialLocationCalculatorConfigData> configData;
    configData.reserve(roiSpecs.size());
    for(const auto& spec : roiSpecs) {
        dai::SpatialLocationCalculatorConfigData cfg;
        cfg.roi = spec.roi;
        cfg.depthThresholds.lowerThreshold = 0;
        cfg.depthThresholds.upperThreshold = 10000;
        cfg.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::AVERAGE;
        cfg.stepSize = 1;
        configData.push_back(cfg);
    }
    spatial->initialConfig->setROIs(configData);

    auto depthQueue = spatial->inputDepth.createInputQueue();
    auto outputQueue = spatial->out.createOutputQueue();
    auto passthroughQueue = spatial->passthroughDepth.createOutputQueue();

    std::vector<std::uint16_t> depthPixels(width * height, 1000);
    auto setRegionDepth = [&](const RoiSpec& spec) {
        const int x0 = static_cast<int>(spec.roi.x);
        const int y0 = static_cast<int>(spec.roi.y);
        for(int y = 0; y < static_cast<int>(spec.heightPx); ++y) {
            for(int x = 0; x < static_cast<int>(spec.widthPx); ++x) {
                depthPixels[(y0 + y) * width + (x0 + x)] = spec.depth;
            }
        }
    };
    for(const auto& spec : roiSpecs) {
        setRegionDepth(spec);
    }

    // Prepare synthetic depth frame
    auto depthFrame = std::make_shared<dai::ImgFrame>();
    depthFrame->setType(dai::ImgFrame::Type::RAW16);
    depthFrame->setSize(width, height);
    depthFrame->setSourceSize(width, height);
    std::vector<std::uint8_t> rawDepth(depthPixels.size() * sizeof(std::uint16_t));
    std::memcpy(rawDepth.data(), depthPixels.data(), rawDepth.size());
    depthFrame->setData(rawDepth);

    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.5F, 0.0F, 3.2F}},
        {{0.0F, 4.8F, 2.4F}},
        {{0.0F, 0.0F, 1.0F}},
    }};
    depthFrame->transformation.setSourceSize(width, height);
    depthFrame->transformation.setSize(width, height);
    depthFrame->transformation.setIntrinsicMatrix(intrinsics);
    REQUIRE(depthFrame->validateTransformations());

    pipeline.start();
    depthQueue->send(depthFrame);

    auto spatialData = outputQueue->get<dai::SpatialLocationCalculatorData>();
    REQUIRE(spatialData != nullptr);
    const auto results = spatialData->getSpatialLocations();
    REQUIRE(results.size() == roiSpecs.size());

    auto passthroughFrame = passthroughQueue->get<dai::ImgFrame>();
    REQUIRE(passthroughFrame != nullptr);
    CHECK(passthroughFrame->getWidth() == width);
    CHECK(passthroughFrame->getHeight() == height);

    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];

    for(std::size_t idx = 0; idx < roiSpecs.size(); ++idx) {
        const auto& spec = roiSpecs[idx];
        const auto& spatialLoc = results.at(idx);

        CHECK(spatialLoc.depthAverage == Approx(static_cast<float>(spec.depth)).margin(1.0F));
        CHECK(spatialLoc.depthMin == spec.depth);
        CHECK(spatialLoc.depthMax == spec.depth);
        CHECK(spatialLoc.depthAveragePixelCount == spec.widthPx * spec.heightPx);

        const float centerX = spec.roi.x + spec.roi.width / 2.0F;
        const float centerY = spec.roi.y + spec.roi.height / 2.0F;
        const float expectedX = ((centerX - cx) / fx) * static_cast<float>(spec.depth);
        const float expectedY = ((centerY - cy) / fy) * static_cast<float>(spec.depth);
        const float expectedZ = static_cast<float>(spec.depth);

        CHECK(spatialLoc.spatialCoordinates.x == Approx(expectedX).margin(5.0F));
        CHECK(spatialLoc.spatialCoordinates.y == Approx(expectedY).margin(5.0F));
        CHECK(spatialLoc.spatialCoordinates.z == Approx(expectedZ).margin(1.0F));
    }

    pipeline.stop();
    pipeline.wait();
}
