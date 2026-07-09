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

namespace dai::node {

struct DepthTestAccess {
    static std::pair<Depth::Algorithm, Depth::Config> selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                                    float targetFps,
                                                                    const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                                    const std::vector<DeviceModelZoo>& modelFilter = {}) {
        const auto selection = Depth::selectBackend(resolution, targetFps, supportedAlgorithms, modelFilter);
        return {selection.algorithm, selection.config};
    }
};

}  // namespace dai::node

namespace {

const std::vector<Depth::Algorithm> kAllRvc4Backends = {
    Depth::Algorithm::NEURAL,
    Depth::Algorithm::NEURAL_ASSISTED_STEREO,
    Depth::Algorithm::STEREO,
    Depth::Algorithm::GPU_STEREO,
};

const std::vector<Depth::Algorithm> kStereoOnly = {Depth::Algorithm::STEREO};

struct Selection {
    Depth::Algorithm algorithm;
    Depth::Config config;
};

Selection select(std::optional<std::pair<uint32_t, uint32_t>> res,
                 float fps,
                 const std::vector<Depth::Algorithm>& supported = kAllRvc4Backends,
                 const std::vector<DeviceModelZoo>& modelFilter = {}) {
    const auto selected = dai::node::DepthTestAccess::selectBackend(res, fps, supported, modelFilter);
    return {selected.first, selected.second};
}

}  // namespace

TEST_CASE("Depth::selectBackend picks NeuralDepth model whose tensor band fits the user resolution", "[Depth]") {
    struct Case {
        uint32_t w, h;
        float fps;
        DeviceModelZoo expectedModel;
    };
    const std::vector<Case> cases = {
        {640, 400, 30.f, DeviceModelZoo::NEURAL_DEPTH_576X360},
        {640, 400, 10.f, DeviceModelZoo::NEURAL_DEPTH_864X540},
        {640, 400, 15.f, DeviceModelZoo::NEURAL_DEPTH_864X540},
        {1280, 800, 1.5f, DeviceModelZoo::NEURAL_DEPTH_1248X780},
        {1280, 800, 5.f, DeviceModelZoo::NEURAL_DEPTH_1248X780},
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
    // 1280x800: no NEURAL tensor band fits; NAS @55 FPS is the first matching profile @30 FPS (required 27).
    const auto sel = select(std::make_pair(1280u, 800u), 30.f);
    REQUIRE(sel.algorithm == Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE(std::holds_alternative<std::monostate>(sel.config));
}

TEST_CASE("Depth::selectBackend picks GPUStereo via resolution-fit fallback when target FPS is not met", "[Depth]") {
    SECTION("oversized resolution falls back to highest-FPS profile that fits") {
        // 1920x1440 exceeds every non-GPU backend cap; GPUStereo 2592x1944 @5 FPS is the resolution-fit fallback.
        const auto sel = select(std::make_pair(1920u, 1440u), 30.f);
        REQUIRE(sel.algorithm == Depth::Algorithm::GPU_STEREO);
        REQUIRE(std::holds_alternative<std::monostate>(sel.config));
    }
    SECTION("native sensor resolution above 1280x800 uses the same fallback path") {
        const auto sel = select(std::make_pair(2592u, 1944u), 30.f);
        REQUIRE(sel.algorithm == Depth::Algorithm::GPU_STEREO);
        REQUIRE(std::holds_alternative<std::monostate>(sel.config));
    }
}

TEST_CASE("Depth::selectBackend falls back to FAST_ACCURACY when no backend covers the resolution at all", "[Depth]") {
    const std::vector<Depth::Algorithm> noGpu = {Depth::Algorithm::NEURAL, Depth::Algorithm::NEURAL_ASSISTED_STEREO, Depth::Algorithm::STEREO};
    // 1920x1440 exceeds every non-GPU backend's resolution cap; without GPUStereo the last-resort fallback kicks in.
    const auto sel = select(std::make_pair(1920u, 1440u), 30.f, noGpu);
    REQUIRE(sel.algorithm == Depth::Algorithm::STEREO);
    REQUIRE(std::get<StereoDepth::PresetMode>(sel.config) == StereoDepth::PresetMode::FAST_ACCURACY);
}

TEST_CASE("Depth::selectBackend picks GPUStereo when it is the only backend that fits both FPS and resolution", "[Depth]") {
    const std::vector<Depth::Algorithm> gpuOnly = {Depth::Algorithm::GPU_STEREO};
    SECTION("1280x800 @30fps -> GPUStereo 1280x800 profile") {
        const auto sel = select(std::make_pair(1280u, 800u), 30.f, gpuOnly);
        REQUIRE(sel.algorithm == Depth::Algorithm::GPU_STEREO);
        REQUIRE(std::holds_alternative<std::monostate>(sel.config));
    }
    SECTION("640x400 @34fps -> GPUStereo 640x400 profile (1280x800@30 fails the FPS gate)") {
        // requiredFps = 34 * 0.9 = 30.6; GPUStereo 1280x800@30 fails, 640x400@55 wins.
        const auto sel = select(std::make_pair(640u, 400u), 34.f, gpuOnly);
        REQUIRE(sel.algorithm == Depth::Algorithm::GPU_STEREO);
        REQUIRE(std::holds_alternative<std::monostate>(sel.config));
    }
}

TEST_CASE("Depth::selectBackend rejects NeuralAssistedStereo above its 1280x800 cap", "[Depth]") {
    const std::vector<Depth::Algorithm> nasOnly = {Depth::Algorithm::NEURAL_ASSISTED_STEREO};
    // 1280x1000 exceeds NAS height cap (800).
    const auto sel = select(std::make_pair(1280u, 1000u), 30.f, nasOnly);
    REQUIRE(sel.algorithm == Depth::Algorithm::STEREO);
    REQUIRE(std::get<StereoDepth::PresetMode>(sel.config) == StereoDepth::PresetMode::FAST_ACCURACY);
}

TEST_CASE("Depth::selectBackend without resolution picks largest NeuralDepth meeting FPS", "[Depth]") {
    REQUIRE(std::get<DeviceModelZoo>(select(std::nullopt, 5.f).config) == DeviceModelZoo::NEURAL_DEPTH_1248X780);
    REQUIRE(std::get<DeviceModelZoo>(select(std::nullopt, 30.f).config) == DeviceModelZoo::NEURAL_DEPTH_576X360);
}

TEST_CASE("Depth::selectBackend without NeuralDepth falls through to StereoDepth presets by quality and FPS", "[Depth]") {
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 15.f, kStereoOnly).config) == StereoDepth::PresetMode::DEFAULT);
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 30.f, kStereoOnly).config) == StereoDepth::PresetMode::FAST_ACCURACY);
    REQUIRE(std::get<StereoDepth::PresetMode>(select(std::nullopt, 60.f, kStereoOnly).config) == StereoDepth::PresetMode::FAST_ACCURACY);
}

TEST_CASE("Depth::selectBackend honors supportedModels filter for NEURAL rows", "[Depth]") {
    const std::vector<DeviceModelZoo> only = {DeviceModelZoo::NEURAL_DEPTH_480X300, DeviceModelZoo::NEURAL_DEPTH_192X120};
    const auto sel = select(std::make_pair(640u, 400u), 30.f, kAllRvc4Backends, only);
    REQUIRE(sel.algorithm == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(sel.config) == DeviceModelZoo::NEURAL_DEPTH_480X300);
}
