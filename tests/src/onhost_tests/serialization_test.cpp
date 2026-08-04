#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Roundtrip") {
    dai::Pipeline p;
    auto stereo = p.create<dai::node::StereoDepth>();

    // Create ground truth properties
    stereo->setInputResolution(0xa0a0a0a0, 0xa0a0a0a0);
    stereo->setOutputSize(0x55555555, 0x55555555);
    stereo->setExtendedDisparity(true);
    stereo->properties.numFramesPool = 42;

    // Round trip
    {
        auto ser = dai::utility::serialize(stereo->properties);
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }

    // Round trip through pipeline
    {
        auto ser = p.getPipelineSchema().nodes[0].properties;
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }
}

TEST_CASE("StereoDepth DSP_RVC2_DEFAULT_64 only changes the DEFAULT disparity width") {
    dai::Pipeline pipeline;
    auto stereoDefault = pipeline.create<dai::node::StereoDepth>();
    auto stereoDefault64 = pipeline.create<dai::node::StereoDepth>();

    stereoDefault->setStereoBackend(dai::StereoDepthProperties::StereoBackend::DSP_RVC2_DEFAULT);
    stereoDefault64->setStereoBackend(dai::StereoDepthProperties::StereoBackend::DSP_RVC2_DEFAULT_64);

    using DisparityWidth = dai::StereoDepthConfig::CostMatching::DisparityWidth;
    REQUIRE(stereoDefault->initialConfig->costMatching.disparityWidth == DisparityWidth::DISPARITY_96);
    REQUIRE(stereoDefault64->initialConfig->costMatching.disparityWidth == DisparityWidth::DISPARITY_64);

    auto defaultConfig = *stereoDefault->initialConfig;
    auto default64Config = *stereoDefault64->initialConfig;
    default64Config.costMatching.disparityWidth = DisparityWidth::DISPARITY_96;
    REQUIRE(dai::utility::serialize(defaultConfig) == dai::utility::serialize(default64Config));

    REQUIRE(stereoDefault->properties.numPostProcessingShaves == stereoDefault64->properties.numPostProcessingShaves);
    REQUIRE(stereoDefault->properties.numPostProcessingMemorySlices == stereoDefault64->properties.numPostProcessingMemorySlices);

    auto serialized = dai::utility::serialize(stereoDefault64->properties);
    dai::StereoDepthProperties deserialized;
    dai::utility::deserialize(serialized, deserialized);
    REQUIRE(deserialized.stereoBackend == dai::StereoDepthProperties::StereoBackend::DSP_RVC2_DEFAULT_64);
    REQUIRE(deserialized.initialConfig.costMatching.disparityWidth == DisparityWidth::DISPARITY_64);
}
