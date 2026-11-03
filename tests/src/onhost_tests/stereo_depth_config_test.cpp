#include <catch2/catch_all.hpp>
#include <stdexcept>
#include <tuple>

#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {
using Config = dai::StereoDepthConfig;
using Filter = Config::PostProcessing::Filter;

Config temporalConfig(int bits = 3) {
    Config config;
    config.setSubpixel(true).setSubpixelFractionalBits(bits).setMedianFilter(Config::MedianFilter::MEDIAN_OFF);
    config.postProcessing.temporalFilter.enable = true;
    config.postProcessing.spatialFilter.enable = false;
    config.postProcessing.speckleFilter.enable = false;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    config.postProcessing.filteringOrder = {Filter::MEDIAN, Filter::DECIMATION, Filter::SPECKLE, Filter::SPATIAL, Filter::TEMPORAL};
    return config;
}
}  // namespace

TEST_CASE("RVC2 disparity encoding matches measured firmware units", "[StereoDepthConfig]") {
    // The 3- and 5-bit encodings were verified against synchronized RVC2
    // disparity/device-depth frames. The 4-bit case checks integer rounding.
    const auto encoding = GENERATE(std::make_tuple(3, 80.0f, 7600.0f), std::make_tuple(4, 80.0f, 7600.0f), std::make_tuple(5, 64.0f, 6080.0f));
    const auto config = temporalConfig(std::get<0>(encoding));
    REQUIRE(config.getDisparityScale() == std::get<1>(encoding));
    REQUIRE(config.getMaxDisparity() == std::get<2>(encoding));
}

TEST_CASE("RVC2 disparity encoding follows the last enabled filter", "[StereoDepthConfig]") {
    auto config = temporalConfig();
    SECTION("Disabled post-processing retains subpixel units") {
        config.postProcessing.temporalFilter.enable = false;
        REQUIRE(config.getDisparityScale() == 8.0f);
        REQUIRE(config.getMaxDisparity() == 760.0f);
    }
    SECTION("Terminal median retains subpixel units") {
        config.setMedianFilter(Config::MedianFilter::KERNEL_7x7);
        config.postProcessing.filteringOrder = {Filter::TEMPORAL, Filter::MEDIAN, Filter::NONE, Filter::NONE, Filter::NONE};
        REQUIRE(config.getDisparityScale() == 8.0f);
        REQUIRE(config.getMaxDisparity() == 760.0f);
    }
    SECTION("Median before temporal retains normalized units") {
        config.setMedianFilter(Config::MedianFilter::KERNEL_7x7);
        REQUIRE(config.getDisparityScale() == 80.0f);
        REQUIRE(config.getMaxDisparity() == 7600.0f);
    }
    SECTION("Firmware disables median when its input exceeds 10 bits") {
        config.setSubpixelFractionalBits(5).setMedianFilter(Config::MedianFilter::KERNEL_7x7);
        config.postProcessing.filteringOrder = {Filter::TEMPORAL, Filter::MEDIAN, Filter::NONE, Filter::NONE, Filter::NONE};
        REQUIRE(config.getDisparityScale() == 64.0f);
        REQUIRE(config.getMaxDisparity() == 6080.0f);
    }
    SECTION("Decimation alone enables normalization") {
        config.postProcessing.temporalFilter.enable = false;
        config.postProcessing.decimationFilter.decimationFactor = 2;
        REQUIRE(config.getDisparityScale() == 80.0f);
        REQUIRE(config.getMaxDisparity() == 7600.0f);
    }
}

TEST_CASE("RVC2 disparity encoding includes matcher precision and search width", "[StereoDepthConfig]") {
    auto config = temporalConfig();
    SECTION("64 disparities") {
        config.costMatching.disparityWidth = Config::CostMatching::DisparityWidth::DISPARITY_64;
        REQUIRE(config.getDisparityScale() == 128.0f);
        REQUIRE(config.getMaxDisparity() == 8064.0f);
    }
    SECTION("Integer disparity") {
        config.setSubpixel(false);
        REQUIRE(config.getDisparityScale() == 86.0f);
        REQUIRE(config.getMaxDisparity() == 8170.0f);
    }
    SECTION("Extended disparity") {
        config.setExtendedDisparity(true);
        REQUIRE(config.getDisparityScale() == 40.0f);
        REQUIRE(config.getMaxDisparity() == 7600.0f);
    }
    SECTION("Disparity shift does not change the encoded range") {
        config.algorithmControl.disparityShift = 48;
        REQUIRE(config.getDisparityScale() == 80.0f);
        REQUIRE(config.getMaxDisparity() == 7600.0f);
    }
    SECTION("Companding cannot be represented by a linear pixel scale") {
        config.costMatching.enableCompanding = true;
        REQUIRE(config.getMaxDisparity() == 7600.0f);
        REQUIRE_THROWS_AS(config.getDisparityScale(), std::invalid_argument);
    }
}

TEST_CASE("RVC2 disparity encoding rejects invalid subpixel shifts", "[StereoDepthConfig]") {
    auto config = temporalConfig();
    config.algorithmControl.subpixelFractionalBits = GENERATE(0, 2, 6, 31);
    REQUIRE_THROWS_AS(config.getDisparityScale(), std::invalid_argument);
    REQUIRE_THROWS_AS(config.getMaxDisparity(), std::invalid_argument);
}

TEST_CASE("RVC2 disparity encoding survives config serialization", "[StereoDepthConfig]") {
    auto config = temporalConfig(5);
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype;
    config.serialize(metadata, datatype);
    Config decoded;
    dai::utility::deserialize(metadata, decoded);
    REQUIRE(datatype == dai::DatatypeEnum::StereoDepthConfig);
    REQUIRE(decoded.getDisparityScale() == 64.0f);
    REQUIRE(decoded.getMaxDisparity() == 6080.0f);
}
