#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include <optional>
#include <utility>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

using dai::DeviceModelZoo;
using dai::node::Depth;
using dai::node::StereoDepth;

namespace {

const std::vector<Depth::Algorithm> kAllRvc4Backends = {
    Depth::Algorithm::NEURAL,
    Depth::Algorithm::NEURAL_ASSISTED_STEREO,
    Depth::Algorithm::STEREO,
    Depth::Algorithm::GPU_STEREO,
};

const std::vector<Depth::Algorithm> kStereoOnly = {Depth::Algorithm::STEREO};

Depth::Selection select(std::optional<std::pair<uint32_t, uint32_t>> res,
                        float fps,
                        const std::vector<Depth::Algorithm>& supported = kAllRvc4Backends) {
    return Depth::selectBackend(res, fps, supported);
}

}  // namespace

TEST_CASE("Depth::selectBackend picks NeuralDepth model whose tensor band fits the user resolution", "[Depth]") {
    struct Case {
        uint32_t w, h;
        float fps;
        DeviceModelZoo expectedModel;
    };
    const std::vector<Case> cases = {
        {640, 400, 30.f, DeviceModelZoo::NEURAL_DEPTH_480X300},
        {640, 400, 10.f, DeviceModelZoo::NEURAL_DEPTH_768X480},
        {640, 400, 15.f, DeviceModelZoo::NEURAL_DEPTH_576X360},
        {1280, 800, 1.5f, DeviceModelZoo::NEURAL_DEPTH_1248X780},
        {1280, 800, 5.f, DeviceModelZoo::NEURAL_DEPTH_960X600},
        {192, 120, 30.f, DeviceModelZoo::NEURAL_DEPTH_192X120},
    };
    for(const auto& c : cases) {
        INFO("case " << c.w << "x" << c.h << " @" << c.fps);
        const auto sel = select(std::make_pair(c.w, c.h), c.fps);
        REQUIRE(sel.algorithm == Depth::Algorithm::NEURAL);
        REQUIRE(std::get<DeviceModelZoo>(sel.config) == c.expectedModel);
    }
}

TEST_CASE("Depth::selectBackend falls back to NAS when no NeuralDepth model meets FPS but resolution fits stereo", "[Depth]") {
    // 640x400 @60: every NEURAL row at ≥51fps has a tensor band that does not fit 640x400, so NAS is the next pick.
    const auto sel = select(std::make_pair(640u, 400u), 60.f);
    REQUIRE(sel.algorithm == Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE(std::holds_alternative<std::monostate>(sel.config));
}

TEST_CASE("Depth::selectBackend forces GPUStereo when resolution exceeds 1280x1280", "[Depth]") {
    const auto sel = select(std::make_pair(1920u, 1440u), 30.f);
    REQUIRE(sel.algorithm == Depth::Algorithm::GPU_STEREO);
    REQUIRE(std::holds_alternative<std::monostate>(sel.config));
}

TEST_CASE("Depth::selectBackend without resolution picks largest NeuralDepth meeting FPS", "[Depth]") {
    REQUIRE(std::get<DeviceModelZoo>(select(std::nullopt, 5.f).config) == DeviceModelZoo::NEURAL_DEPTH_960X600);
    REQUIRE(std::get<DeviceModelZoo>(select(std::nullopt, 1.f).config) == DeviceModelZoo::NEURAL_DEPTH_1248X780);
    REQUIRE(std::get<DeviceModelZoo>(select(std::nullopt, 30.f).config) == DeviceModelZoo::NEURAL_DEPTH_480X300);
}

TEST_CASE("Depth::selectBackend without NeuralDepth falls through to StereoDepth presets by quality and FPS", "[Depth]") {
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 15.f, kStereoOnly).config) == StereoDepth::PresetMode::ACCURACY);
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 30.f, kStereoOnly).config) == StereoDepth::PresetMode::DENSITY);
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 60.f, kStereoOnly).config) == StereoDepth::PresetMode::FAST_DENSITY);
}

TEST_CASE("Depth::selectBackend honors supportedModels filter for NEURAL rows", "[Depth]") {
    const std::vector<DeviceModelZoo> only = {DeviceModelZoo::NEURAL_DEPTH_480X300, DeviceModelZoo::NEURAL_DEPTH_192X120};
    const auto sel = Depth::selectBackend(std::make_pair(640u, 400u), 30.f, kAllRvc4Backends, only);
    REQUIRE(sel.algorithm == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(sel.config) == DeviceModelZoo::NEURAL_DEPTH_480X300);
}

TEST_CASE("Depth::exceedsStereoDepthMaxResolution", "[Depth]") {
    REQUIRE_FALSE(Depth::exceedsStereoDepthMaxResolution(1280, 1280));
    REQUIRE(Depth::exceedsStereoDepthMaxResolution(1281, 800));
    REQUIRE(Depth::exceedsStereoDepthMaxResolution(800, 1281));
}
