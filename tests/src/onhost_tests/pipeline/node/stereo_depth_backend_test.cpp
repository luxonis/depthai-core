#include <array>
#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>

namespace dai::node {

struct StereoDepthTestAccess {
    static std::shared_ptr<StereoDepth> create() {
        return std::shared_ptr<StereoDepth>(new StereoDepth(std::make_unique<StereoDepth::Properties>()));
    }

    static void preparePropertiesForPlatform(StereoDepth& node, Platform platform) {
        node.preparePropertiesForPlatform(platform);
    }

    static void setPresetForPlatform(StereoDepth& node, Platform platform, StereoDepth::PresetMode mode) {
        node.setProfilePresetForPlatform(platform, mode);
    }

    static void setRvc2Preset(StereoDepth& node, StereoDepth::PresetMode mode) {
        node.setRvc2ProfilePreset(mode);
    }

    static void setRvc4Preset(StereoDepth& node, StereoDepth::PresetMode mode) {
        node.setRvc4ProfilePreset(mode);
    }

    static StereoDepth::Properties& getProperties(StereoDepth& node) {
        return node.getProperties();
    }
};

}  // namespace dai::node

namespace {

using Backend = dai::StereoDepthProperties::StereoBackend;
using DisparityWidth = dai::StereoDepthConfig::CostMatching::DisparityWidth;
using PresetMode = dai::node::StereoDepth::PresetMode;

constexpr std::array PRESETS = {PresetMode::FAST_ACCURACY,
                                PresetMode::FAST_DENSITY,
                                PresetMode::DEFAULT,
                                PresetMode::FACE,
                                PresetMode::HIGH_DETAIL,
                                PresetMode::ROBOTICS,
                                PresetMode::DENSITY,
                                PresetMode::ACCURACY};
constexpr std::array DISPARITY_WIDTHS = {DisparityWidth::DISPARITY_64, DisparityWidth::DISPARITY_96};

struct NodeState {
    std::vector<std::uint8_t> config;
    int shaves;
    int memorySlices;
};

std::shared_ptr<dai::node::StereoDepth> createStereo() {
    return dai::node::StereoDepthTestAccess::create();
}

NodeState stateOf(dai::node::StereoDepth& stereo) {
    return {dai::utility::serialize(*stereo.initialConfig), stereo.properties.numPostProcessingShaves, stereo.properties.numPostProcessingMemorySlices};
}

void requireSameState(dai::node::StereoDepth& actual, dai::node::StereoDepth& expected) {
    const auto actualState = stateOf(actual);
    const auto expectedState = stateOf(expected);
    REQUIRE(actualState.config == expectedState.config);
    REQUIRE(actualState.shaves == expectedState.shaves);
    REQUIRE(actualState.memorySlices == expectedState.memorySlices);
}

std::uint64_t fnv1a64(const std::vector<std::uint8_t>& bytes) {
    std::uint64_t hash = 14695981039346656037ULL;
    for(const auto byte : bytes) {
        hash ^= byte;
        hash *= 1099511628211ULL;
    }
    return hash;
}

struct GoldenPresetState {
    std::size_t serializedSize;
    std::uint64_t serializedFnv1a;
    int shaves;
    int memorySlices;
};

// Frozen from origin/develop f0f9956f9a35 by serializing each host-side RVC2
// preset at D64 and D96. Physical output parity is covered separately by HIL;
// these values keep the expected config independent from the helper under test.
constexpr std::array<std::array<GoldenPresetState, 2>, 8> RVC2_PRESET_GOLDENS = {{
    {{{145, 1586794784019944552ULL, -1, -1}, {145, 7775341888342253305ULL, -1, -1}}},
    {{{144, 10426678569622650863ULL, -1, -1}, {144, 2042099752392850274ULL, -1, -1}}},
    {{{143, 17343518243872621592ULL, 3, 3}, {143, 8029195657124377317ULL, 3, 3}}},
    {{{143, 15072195451843779047ULL, 3, 3}, {143, 1374677143043330458ULL, 3, 3}}},
    {{{144, 265321131344157735ULL, 3, 3}, {144, 18129037989369456214ULL, 3, 3}}},
    {{{143, 6339175449969610470ULL, 3, 3}, {143, 12643459460346317859ULL, 3, 3}}},
    {{{144, 10426678569622650863ULL, -1, -1}, {144, 2042099752392850274ULL, -1, -1}}},
    {{{145, 1586794784019944552ULL, -1, -1}, {145, 7775341888342253305ULL, -1, -1}}},
}};

}  // namespace

TEST_CASE("RVC4 RVC2 backend presets match frozen RVC2 serialization states", "[StereoDepth]") {
    for(std::size_t presetIndex = 0; presetIndex < PRESETS.size(); ++presetIndex) {
        for(std::size_t widthIndex = 0; widthIndex < DISPARITY_WIDTHS.size(); ++widthIndex) {
            CAPTURE(presetIndex, widthIndex);
            auto stereo = createStereo();
            stereo->initialConfig->costMatching.disparityWidth = DISPARITY_WIDTHS[widthIndex];
            stereo->setStereoBackend(Backend::RVC2);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*stereo, dai::Platform::RVC4, PRESETS[presetIndex]);
            const auto state = stateOf(*stereo);
            const auto& golden = RVC2_PRESET_GOLDENS[presetIndex][widthIndex];
            REQUIRE(state.config.size() == golden.serializedSize);
            REQUIRE(fnv1a64(state.config) == golden.serializedFnv1a);
            REQUIRE(state.shaves == golden.shaves);
            REQUIRE(state.memorySlices == golden.memorySlices);
        }
    }
}

TEST_CASE("Stereo backend selection changes no configuration or resources", "[StereoDepth]") {
    auto stereo = createStereo();
    stereo->initialConfig->costMatching.disparityWidth = DisparityWidth::DISPARITY_64;
    stereo->initialConfig->setConfidenceThreshold(37);
    stereo->initialConfig->postProcessing.speckleFilter.enable = true;
    stereo->setPostProcessingHardwareResources(7, 5);
    const auto initial = stateOf(*stereo);

    for(const auto backend : {Backend::RVC2, Backend::EVA, Backend::RVC2}) {
        stereo->setStereoBackend(backend);
        const auto selected = stateOf(*stereo);
        REQUIRE(selected.config == initial.config);
        REQUIRE(selected.shaves == initial.shaves);
        REQUIRE(selected.memorySlices == initial.memorySlices);
    }
}

TEST_CASE("First RVC4 preset after backend selection discards stale custom state", "[StereoDepth]") {
    auto expected = createStereo();
    expected->initialConfig->costMatching.disparityWidth = DisparityWidth::DISPARITY_64;
    expected->setStereoBackend(Backend::RVC2);
    dai::node::StereoDepthTestAccess::setPresetForPlatform(*expected, dai::Platform::RVC4, PresetMode::DEFAULT);

    auto actual = createStereo();
    actual->initialConfig->costMatching.disparityWidth = DisparityWidth::DISPARITY_64;
    actual->initialConfig->postProcessing.brightnessFilter.minBrightness = 42;
    actual->setPostProcessingHardwareResources(7, 5);
    actual->setStereoBackend(Backend::RVC2);
    dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, PresetMode::DEFAULT);

    requireSameState(*actual, *expected);
}

TEST_CASE("RVC4 RVC2 backend uses every native RVC2 preset without changing disparity range", "[StereoDepth]") {
    for(const auto preset : PRESETS) {
        for(const auto width : DISPARITY_WIDTHS) {
            CAPTURE(static_cast<int>(preset), static_cast<int>(width));

            auto expected = createStereo();
            expected->initialConfig->costMatching.disparityWidth = width;
            dai::node::StereoDepthTestAccess::setRvc2Preset(*expected, preset);

            auto actual = createStereo();
            actual->initialConfig->costMatching.disparityWidth = width;
            actual->setStereoBackend(Backend::RVC2);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, preset);

            requireSameState(*actual, *expected);
            REQUIRE(actual->initialConfig->costMatching.disparityWidth == width);
        }
    }
}

TEST_CASE("Backend selection is independent of RVC2 preset call order", "[StereoDepth]") {
    for(const auto first : PRESETS) {
        for(const auto second : PRESETS) {
            CAPTURE(static_cast<int>(first), static_cast<int>(second));

            auto expected = createStereo();
            dai::node::StereoDepthTestAccess::setRvc2Preset(*expected, first);
            dai::node::StereoDepthTestAccess::setRvc2Preset(*expected, second);

            auto actual = createStereo();
            actual->setStereoBackend(Backend::RVC2);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, first);
            actual->setStereoBackend(Backend::EVA);
            actual->setStereoBackend(Backend::RVC2);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, second);

            requireSameState(*actual, *expected);
        }
    }
}

TEST_CASE("EVA preset configuration is unchanged by backend selection", "[StereoDepth]") {
    for(const auto preset : PRESETS) {
        CAPTURE(static_cast<int>(preset));

        auto expected = createStereo();
        dai::node::StereoDepthTestAccess::setRvc4Preset(*expected, preset);

        auto actual = createStereo();
        actual->setStereoBackend(Backend::RVC2);
        actual->setStereoBackend(Backend::EVA);
        dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, preset);

        requireSameState(*actual, *expected);
    }
}

TEST_CASE("Cross-backend RVC4 presets start from a fresh target preset", "[StereoDepth]") {
    for(const auto sourceBackend : {Backend::RVC2, Backend::EVA}) {
        const auto targetBackend = sourceBackend == Backend::RVC2 ? Backend::EVA : Backend::RVC2;
        for(const auto sourcePreset : PRESETS) {
            for(const auto targetPreset : PRESETS) {
                for(const auto width : DISPARITY_WIDTHS) {
                    CAPTURE(static_cast<int>(sourceBackend), static_cast<int>(sourcePreset), static_cast<int>(targetPreset), static_cast<int>(width));

                    auto expected = createStereo();
                    expected->initialConfig->costMatching.disparityWidth = width;
                    expected->setStereoBackend(targetBackend);
                    dai::node::StereoDepthTestAccess::setPresetForPlatform(*expected, dai::Platform::RVC4, targetPreset);

                    auto actual = createStereo();
                    actual->initialConfig->costMatching.disparityWidth = width;
                    actual->setStereoBackend(sourceBackend);
                    dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, sourcePreset);
                    const auto sourceState = stateOf(*actual);

                    actual->setStereoBackend(targetBackend);
                    const auto selectedState = stateOf(*actual);
                    REQUIRE(selectedState.config == sourceState.config);
                    REQUIRE(selectedState.shaves == sourceState.shaves);
                    REQUIRE(selectedState.memorySlices == sourceState.memorySlices);

                    dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, targetPreset);
                    requireSameState(*actual, *expected);
                    REQUIRE(actual->initialConfig->costMatching.disparityWidth == width);
                }
            }
        }
    }
}

TEST_CASE("Same-backend EVA presets retain their overlay semantics", "[StereoDepth]") {
    for(const auto first : PRESETS) {
        for(const auto second : PRESETS) {
            CAPTURE(static_cast<int>(first), static_cast<int>(second));

            auto expected = createStereo();
            dai::node::StereoDepthTestAccess::setRvc4Preset(*expected, first);
            dai::node::StereoDepthTestAccess::setRvc4Preset(*expected, second);

            auto actual = createStereo();
            actual->setStereoBackend(Backend::EVA);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, first);
            actual->setStereoBackend(Backend::EVA);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC4, second);

            requireSameState(*actual, *expected);
        }
    }
}

TEST_CASE("Physical RVC2 preset calls retain native overlay semantics", "[StereoDepth]") {
    for(const auto first : PRESETS) {
        for(const auto second : PRESETS) {
            CAPTURE(static_cast<int>(first), static_cast<int>(second));

            auto expected = createStereo();
            dai::node::StereoDepthTestAccess::setRvc2Preset(*expected, first);
            dai::node::StereoDepthTestAccess::setRvc2Preset(*expected, second);

            auto actual = createStereo();
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC2, first);
            dai::node::StereoDepthTestAccess::setPresetForPlatform(*actual, dai::Platform::RVC2, second);

            requireSameState(*actual, *expected);
        }
    }
}

TEST_CASE("Stereo backend and disparity width serialize independently", "[StereoDepth]") {
    for(const auto backend : {Backend::EVA, Backend::RVC2}) {
        for(const auto width : DISPARITY_WIDTHS) {
            CAPTURE(static_cast<int>(backend), static_cast<int>(width));

            auto stereo = createStereo();
            stereo->setStereoBackend(backend);
            stereo->initialConfig->costMatching.disparityWidth = width;
            dai::node::StereoDepthTestAccess::preparePropertiesForPlatform(*stereo, dai::Platform::RVC4);

            const auto serialized = dai::utility::serialize(dai::node::StereoDepthTestAccess::getProperties(*stereo));
            dai::StereoDepthProperties deserialized;
            dai::utility::deserialize(serialized, deserialized);

            REQUIRE(deserialized.enableRuntimeStereoModeSwitch == (backend == Backend::RVC2));
            REQUIRE(deserialized.initialConfig.costMatching.disparityWidth == width);
        }
    }
}

TEST_CASE("RVC2 serialization ignores the host-only backend selector", "[StereoDepth]") {
    auto eva = createStereo();
    eva->setStereoBackend(Backend::EVA);
    dai::node::StereoDepthTestAccess::preparePropertiesForPlatform(*eva, dai::Platform::RVC2);

    auto rvc2 = createStereo();
    rvc2->setStereoBackend(Backend::RVC2);
    dai::node::StereoDepthTestAccess::preparePropertiesForPlatform(*rvc2, dai::Platform::RVC2);

    const auto evaProperties = dai::utility::serialize(dai::node::StereoDepthTestAccess::getProperties(*eva));
    const auto rvc2Properties = dai::utility::serialize(dai::node::StereoDepthTestAccess::getProperties(*rvc2));
    REQUIRE(rvc2Properties == evaProperties);
}
