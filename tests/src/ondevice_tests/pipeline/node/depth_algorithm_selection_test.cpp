#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "utility/CameraFeatures.hpp"

using dai::DeviceModelZoo;
using dai::node::Depth;
using dai::node::StereoDepth;

namespace dai::node {

struct DepthTestAccess {
    static void validateFps(StereoDepth::PresetMode preset, float fps) {
        Depth::validateRequestedFps(Depth::Algorithm::STEREO, preset, fps, std::make_pair(640u, 400u), dai::Version(1, 47, 0));
    }
    static void validateResolution(Depth::Algorithm algorithm, std::optional<std::pair<uint32_t, uint32_t>> resolution) {
        Depth::validateProcessingResolution(algorithm, resolution);
    }
    static DeviceModelZoo modelForOS(const std::optional<dai::Version>& version) {
        const auto selection = Depth::selectBackend(Platform::RVC4, std::make_pair(640u, 400u), 30.f, {Depth::Algorithm::NEURAL}, {}, true, version);
        return std::get<DeviceModelZoo>(selection.config);
    }
    static std::pair<Depth::Algorithm, Depth::Config> selectOptimized(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                                      float fps,
                                                                      const std::vector<Depth::Algorithm>& algorithms,
                                                                      const std::vector<DeviceModelZoo>& models = {},
                                                                      bool strict = false,
                                                                      std::optional<std::pair<uint32_t, uint32_t>> stereoResolution = std::nullopt) {
        const auto selection = Depth::selectBackend(Platform::RVC4, resolution, fps, algorithms, models, strict, dai::Version(1, 47, 0), stereoResolution);
        return {selection.algorithm, selection.config};
    }
    static std::pair<Depth::Algorithm, Depth::Config> selectWithOs(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                                   float fps,
                                                                   const std::vector<Depth::Algorithm>& algorithms,
                                                                   bool strict,
                                                                   const std::optional<dai::Version>& osVersion) {
        const auto selection = Depth::selectBackend(Platform::RVC4, resolution, fps, algorithms, {}, strict, osVersion);
        return {selection.algorithm, selection.config};
    }
    // Test the optional-output contract without connecting a device or building a backend.
    static void setWiredOutputs(Depth& depth, Depth::Algorithm algorithm, Node::Output& depthOutput, Node::Output* left, Node::Output* right) {
        depth.graphBuilt_ = true;
        depth.resolved_.algorithm = algorithm;
        depth.depthOut_ = &depthOutput;
        depth.rectifiedLeftOut_ = left;
        depth.rectifiedRightOut_ = right;
    }

    static std::pair<Depth::Algorithm, Depth::Config> selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                                    float targetFps,
                                                                    const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                                    const std::vector<DeviceModelZoo>& modelFilter = {}) {
        const auto selection = Depth::selectBackend(Platform::RVC4, resolution, targetFps, supportedAlgorithms, modelFilter);
        return {selection.algorithm, selection.config};
    }

    static std::pair<Depth::Algorithm, Depth::Config> selectNonRvc4Backend(const std::vector<Depth::Algorithm>& supportedAlgorithms) {
        const auto selection = Depth::selectBackend(Platform::RVC2, std::nullopt, 30.f, supportedAlgorithms);
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

TEST_CASE("Depth::getSupportedAlgorithms requires a non-null device", "[Depth]") {
    REQUIRE_THROWS_WITH(Depth::getSupportedAlgorithms(nullptr), Catch::Matchers::ContainsSubstring("requires a device"));
}

TEST_CASE("Depth::getDefaultAlgorithm requires a non-null device", "[Depth]") {
    REQUIRE_THROWS_WITH(Depth::getDefaultAlgorithm(nullptr), Catch::Matchers::ContainsSubstring("requires a device"));
}

TEST_CASE("Depth::selectBackend prefers ToF on non-RVC4 devices without requiring stereo", "[Depth]") {
    for(const auto& supported : {std::vector{Depth::Algorithm::TOF}, std::vector{Depth::Algorithm::STEREO, Depth::Algorithm::TOF}}) {
        const auto selected = dai::node::DepthTestAccess::selectNonRvc4Backend(supported);
        REQUIRE(selected.first == Depth::Algorithm::TOF);
        REQUIRE(std::holds_alternative<std::monostate>(selected.second));
    }
}

TEST_CASE("Depth::selectBackend preserves stereo selection on non-RVC4 devices without ToF", "[Depth]") {
    const auto selected = dai::node::DepthTestAccess::selectNonRvc4Backend(kStereoOnly);
    REQUIRE(selected.first == Depth::Algorithm::STEREO);
    REQUIRE(std::get<StereoDepth::PresetMode>(selected.second) == StereoDepth::PresetMode::FAST_ACCURACY);
}

TEST_CASE("Depth::selectBackend selects ToF on RVC4 without stereo", "[Depth]") {
    const auto sel = select(std::nullopt, 30.f, {Depth::Algorithm::TOF});
    REQUIRE(sel.algorithm == Depth::Algorithm::TOF);
    REQUIRE(std::holds_alternative<std::monostate>(sel.config));
}

TEST_CASE("Depth::selectBackend preserves RVC4 stereo selection when ToF is also available", "[Depth]") {
    const auto sel = select(std::make_pair(640u, 400u), 30.f, {Depth::Algorithm::STEREO, Depth::Algorithm::TOF});
    REQUIRE(sel.algorithm == Depth::Algorithm::STEREO);
    REQUIRE(std::get<StereoDepth::PresetMode>(sel.config) == StereoDepth::PresetMode::FAST_ACCURACY);
}

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
    // 1280x800: fitting NEURAL models fail the legacy FPS threshold; NAS @55 FPS is the first matching profile @30 FPS (required 27).
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

TEST_CASE("Depth::selectBackend rejects resolutions no available backend can cover", "[Depth]") {
    const std::vector<Depth::Algorithm> noGpu = {Depth::Algorithm::NEURAL, Depth::Algorithm::NEURAL_ASSISTED_STEREO, Depth::Algorithm::STEREO};
    REQUIRE_THROWS_WITH(select(std::make_pair(1920u, 1440u), 30.f, noGpu), Catch::Matchers::ContainsSubstring("no available algorithm"));
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
    REQUIRE_THROWS_WITH(select(std::make_pair(1280u, 1000u), 30.f, nasOnly), Catch::Matchers::ContainsSubstring("no available algorithm"));
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

TEST_CASE("Depth rectified outputs require a wired backend", "[Depth][RectifiedOutputs]") {
    auto depth = Depth::create();
    REQUIRE_THROWS_WITH(depth->hasRectifiedOutputs(), Catch::Matchers::ContainsSubstring("attach the node to a pipeline"));
    REQUIRE_THROWS(depth->rectifiedLeft());
    REQUIRE_THROWS(depth->rectifiedRight());
}

TEST_CASE("Depth ToF has no rectified pair and keeps its depth output usable", "[Depth][RectifiedOutputs]") {
    auto depth = Depth::create();
    auto stereo = StereoDepth::create();  // Only owns dummy outputs; no hardware connection.
    dai::node::DepthTestAccess::setWiredOutputs(*depth, Depth::Algorithm::TOF, stereo->depth, nullptr, nullptr);
    REQUIRE_FALSE(depth->hasRectifiedOutputs());
    REQUIRE_THROWS_WITH(depth->rectifiedLeft(), Catch::Matchers::ContainsSubstring("unavailable for the TOF backend"));
    REQUIRE_THROWS_WITH(depth->rectifiedRight(), Catch::Matchers::ContainsSubstring("unavailable for the TOF backend"));
    REQUIRE(depth->getRequestedAlgorithm() == Depth::Algorithm::AUTO);
    REQUIRE(depth->getResolvedAlgorithm() == Depth::Algorithm::TOF);
    REQUIRE(&depth->depth() == &stereo->depth);
    REQUIRE(depth->getNodeMap().empty());  // Querying did not create extra stereo nodes.
}

TEST_CASE("Depth rectified access forwards backend outputs and locks configuration", "[Depth][RectifiedOutputs]") {
    auto depth = Depth::create();
    auto stereo = StereoDepth::create();
    dai::node::DepthTestAccess::setWiredOutputs(*depth, Depth::Algorithm::STEREO, stereo->depth, &stereo->rectifiedLeft, &stereo->rectifiedRight);
    REQUIRE(depth->hasRectifiedOutputs());
    REQUIRE(&depth->rectifiedLeft() == &stereo->rectifiedLeft);
    REQUIRE(&depth->rectifiedRight() == &stereo->rectifiedRight);
    REQUIRE(&depth->depth() == &stereo->depth);
    REQUIRE_THROWS(depth->setAlgorithm(Depth::Algorithm::TOF));
}

TEST_CASE("Depth optimized neural profile requires a recognized released OS", "[Depth][FpsProfile]") {
    REQUIRE(dai::node::DepthTestAccess::modelForOS(std::nullopt) == DeviceModelZoo::NEURAL_DEPTH_576X360);
    for(const auto* version : {"1.46.0", "1.47.0", "1.46.1", "1.100.0", "2.0.0", "1.47.0+build", "1.47.0-rc.1"}) {
        INFO(version);
        REQUIRE(dai::node::DepthTestAccess::modelForOS(dai::Version(version)) == DeviceModelZoo::NEURAL_DEPTH_864X540);
    }
    for(const auto* version : {"1.45.99", "1.46.0-rc.1"}) {
        INFO(version);
        REQUIRE(dai::node::DepthTestAccess::modelForOS(dai::Version(version)) == DeviceModelZoo::NEURAL_DEPTH_576X360);
    }
}

TEST_CASE("Depth optimized neural profile selects measured faster models", "[Depth][FpsProfile]") {
    struct Case {
        uint32_t width, height;
        float fps;
        DeviceModelZoo model;
    };
    for(const auto& c : std::vector<Case>{{640, 400, 30.f, DeviceModelZoo::NEURAL_DEPTH_864X540},
                                          {640, 400, 50.f, DeviceModelZoo::NEURAL_DEPTH_864X540},
                                          {640, 400, 60.f, DeviceModelZoo::NEURAL_DEPTH_768X480},
                                          {1280, 800, 15.f, DeviceModelZoo::NEURAL_DEPTH_1248X780},
                                          {1280, 800, 20.f, DeviceModelZoo::NEURAL_DEPTH_1248X780},
                                          {1280, 800, 30.f, DeviceModelZoo::NEURAL_DEPTH_1056X660},
                                          {1280, 800, 40.f, DeviceModelZoo::NEURAL_DEPTH_960X600},
                                          {192, 120, 30.f, DeviceModelZoo::NEURAL_DEPTH_192X120}}) {
        INFO(c.width << "x" << c.height << " @ " << c.fps);
        const auto selected = dai::node::DepthTestAccess::selectOptimized(std::make_pair(c.width, c.height), c.fps, kAllRvc4Backends);
        REQUIRE(selected.first == Depth::Algorithm::NEURAL);
        REQUIRE(std::get<DeviceModelZoo>(selected.second) == c.model);
    }
}

TEST_CASE("Depth optimized profile preserves resolution and model filters", "[Depth][FpsProfile]") {
    const auto native60 = dai::node::DepthTestAccess::selectOptimized(std::make_pair(1280u, 800u), 60.f, kAllRvc4Backends);
    REQUIRE(native60.first == Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    const auto limited =
        dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 30.f, kAllRvc4Backends, {DeviceModelZoo::NEURAL_DEPTH_480X300});
    REQUIRE(std::get<DeviceModelZoo>(limited.second) == DeviceModelZoo::NEURAL_DEPTH_480X300);
    REQUIRE_THROWS(dai::node::DepthTestAccess::selectOptimized(std::make_pair(1280u, 800u), 100.f, kAllRvc4Backends, {}, true));
}

TEST_CASE("Depth optimized profile keeps tolerance and uses faster fallback ranking", "[Depth][FpsProfile]") {
    const auto atBoundary = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 50.f, kAllRvc4Backends);
    REQUIRE(std::get<DeviceModelZoo>(atBoundary.second) == DeviceModelZoo::NEURAL_DEPTH_864X540);
    const auto aboveBoundary = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 50.1f, kAllRvc4Backends);
    REQUIRE(std::get<DeviceModelZoo>(aboveBoundary.second) == DeviceModelZoo::NEURAL_DEPTH_768X480);
    const auto fallback = dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 100.f, kAllRvc4Backends);
    REQUIRE(fallback.first == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(fallback.second) == DeviceModelZoo::NEURAL_DEPTH_768X480);
    const auto stereo = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 30.f, kStereoOnly);
    REQUIRE(stereo.first == Depth::Algorithm::STEREO);
    REQUIRE(std::get<StereoDepth::PresetMode>(stereo.second) == StereoDepth::PresetMode::FAST_ACCURACY);
}

TEST_CASE("Depth optimized neural profiles honor every advertised FPS and tensor band", "[Depth][FpsProfile][adversarial]") {
    struct Profile {
        DeviceModelZoo model;
        uint32_t width;
        uint32_t height;
        float maxFps;
    };
    const std::vector<Profile> profiles = {
        {DeviceModelZoo::NEURAL_DEPTH_1248X780, 1248, 780, 20.f},
        {DeviceModelZoo::NEURAL_DEPTH_1056X660, 1056, 660, 30.f},
        {DeviceModelZoo::NEURAL_DEPTH_960X600, 960, 600, 38.f},
        {DeviceModelZoo::NEURAL_DEPTH_864X540, 864, 540, 45.f},
        {DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480, 60.f},
        {DeviceModelZoo::NEURAL_DEPTH_576X360, 576, 360, 60.f},
        {DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300, 60.f},
        {DeviceModelZoo::NEURAL_DEPTH_384X240, 384, 240, 60.f},
        {DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180, 60.f},
        {DeviceModelZoo::NEURAL_DEPTH_192X120, 192, 120, 60.f},
    };

    for(const auto& p : profiles) {
        INFO("model=" << static_cast<unsigned>(p.model) << " maxFps=" << p.maxFps);
        // Restrict candidates to this supported model so priority cannot mask a bad row.
        // The selection is strict (explicit FPS), so no safety margin applies: the request must fit the model exactly.
        const float targetFps = p.maxFps;
        const uint32_t minW = static_cast<uint32_t>(static_cast<float>(p.width) / 1.4142135f);
        const uint32_t maxW = static_cast<uint32_t>(static_cast<float>(p.width) * 1.4142135f);
        const uint32_t minH = static_cast<uint32_t>(static_cast<float>(p.height) / 1.4142135f);
        const uint32_t maxH = static_cast<uint32_t>(static_cast<float>(p.height) * 1.4142135f);
        for(const auto& resolution : {std::make_pair(minW, minH), std::make_pair(p.width, p.height), std::make_pair(maxW, maxH)}) {
            CAPTURE(resolution.first, resolution.second);
            const auto selected = dai::node::DepthTestAccess::selectOptimized(resolution, targetFps, {Depth::Algorithm::NEURAL}, {p.model}, true);
            REQUIRE(selected.first == Depth::Algorithm::NEURAL);
            REQUIRE(std::get<DeviceModelZoo>(selected.second) == p.model);
        }

        // Above the exact capacity, strict mode must reject this sole available neural row.
        REQUIRE_THROWS(
            dai::node::DepthTestAccess::selectOptimized(std::make_pair(p.width, p.height), p.maxFps + 0.2f, {Depth::Algorithm::NEURAL}, {p.model}, true));
    }
}

TEST_CASE("Depth pre-1.46 OS keeps the legacy model selected at 640x400@30", "[Depth][FpsProfile][adversarial]") {
    REQUIRE(dai::node::DepthTestAccess::modelForOS(dai::Version("1.45.99")) == DeviceModelZoo::NEURAL_DEPTH_576X360);
    const auto legacy = select(std::make_pair(640u, 400u), 30.f);
    REQUIRE(legacy.algorithm == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(legacy.config) == DeviceModelZoo::NEURAL_DEPTH_576X360);

    // With the optimized table, 864x540 supports 45 FPS; the unchanged 0.9 tolerance
    // means it is the first profile satisfying a 30 FPS request at this resolution.
    const auto optimized = dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 30.f, kAllRvc4Backends);
    REQUIRE(optimized.first == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(optimized.second) == DeviceModelZoo::NEURAL_DEPTH_864X540);
}

TEST_CASE("Depth optimized selector rejects impossible strict requests and handles nonfinite FPS conservatively", "[Depth][FpsProfile][adversarial]") {
    const auto neuralOnly = std::vector<Depth::Algorithm>{Depth::Algorithm::NEURAL};
    const auto onlyLargeModel = std::vector<DeviceModelZoo>{DeviceModelZoo::NEURAL_DEPTH_1248X780};

    // Unknown model catalogs must not accidentally select an unsupported neural model.
    REQUIRE_THROWS(
        dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 10.f, neuralOnly, {DeviceModelZoo::TOF_NEURAL_FUSION_672X804}, true));

    // Strict mode is defined for an impossible FPS+resolution pair; ensure the requested resolution
    // is covered by the candidate model so only FPS causes the failure.
    REQUIRE_THROWS(dai::node::DepthTestAccess::selectOptimized(std::make_pair(1280u, 800u), 1000.f, neuralOnly, onlyLargeModel, true));
    REQUIRE_THROWS(dai::node::DepthTestAccess::selectOptimized(std::make_pair(100000u, 100000u), 30.f, neuralOnly, onlyLargeModel, true));

    // Existing targetFpsWithDefault behavior maps NaN, zero, and negative values to the 30 FPS default.
    for(float fps : {std::numeric_limits<float>::quiet_NaN(), 0.f, -1.f, -std::numeric_limits<float>::infinity()}) {
        INFO("fps=" << fps);
        const auto result = dai::node::DepthTestAccess::selectOptimized(std::nullopt, fps, neuralOnly, {DeviceModelZoo::NEURAL_DEPTH_864X540}, true);
        REQUIRE(result.first == Depth::Algorithm::NEURAL);
        REQUIRE(std::get<DeviceModelZoo>(result.second) == DeviceModelZoo::NEURAL_DEPTH_864X540);
    }
    REQUIRE_THROWS(dai::node::DepthTestAccess::selectOptimized(
        std::make_pair(864u, 540u), std::numeric_limits<float>::infinity(), neuralOnly, {DeviceModelZoo::NEURAL_DEPTH_864X540}, true));
}

TEST_CASE("Depth never falls back to an unavailable backend", "[Depth][FpsProfile][adversarial]") {
    const std::vector<Depth::Algorithm> neuralOnly = {Depth::Algorithm::NEURAL};
    const std::vector<DeviceModelZoo> noUsableModel = {DeviceModelZoo::TOF_NEURAL_FUSION_672X804};
    REQUIRE_THROWS_WITH(select(std::make_pair(640u, 400u), 10.f, neuralOnly, noUsableModel), Catch::Matchers::ContainsSubstring("no available algorithm"));
    REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 10.f, neuralOnly, noUsableModel),
                        Catch::Matchers::ContainsSubstring("no available algorithm"));
    REQUIRE_THROWS(select(std::nullopt, 30.f, {}));
    REQUIRE_THROWS(dai::node::DepthTestAccess::selectOptimized(std::nullopt, 30.f, {}));
    // Available EVA still provides the existing best-effort FPS fallback.
    REQUIRE(select(std::make_pair(640u, 400u), 100.f, kStereoOnly).algorithm == Depth::Algorithm::STEREO);
}

TEST_CASE("Pinned AUTO rejects stereo-only hardware below the requested FPS", "[Depth][FpsProfile]") {
    const auto resolution = std::make_pair(640u, 400u);
    const auto bestEffort = dai::node::DepthTestAccess::selectOptimized(resolution, 60.f, kStereoOnly);
    REQUIRE(bestEffort.first == Depth::Algorithm::STEREO);
    REQUIRE_THROWS_AS(dai::node::DepthTestAccess::selectOptimized(resolution, 60.f, kStereoOnly, {}, true), std::runtime_error);
}

TEST_CASE("Explicit FPS above every backend profile is rejected, never downgraded", "[Depth][ExplicitFps]") {
    for(const auto& resolution : {std::optional<std::pair<uint32_t, uint32_t>>{}, std::optional{std::make_pair(640u, 400u)}}) {
        REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::selectOptimized(resolution, 1000.f, kAllRvc4Backends, {}, true),
                            Catch::Matchers::ContainsSubstring("no available algorithm"));
    }
    REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::selectOptimized(std::make_pair(1920u, 1440u), 30.f, kAllRvc4Backends, {}, true),
                        Catch::Matchers::ContainsSubstring("no available algorithm"));
    // Unpinned FPS keeps the best-effort fallback.
    REQUIRE_NOTHROW(dai::node::DepthTestAccess::selectOptimized(std::nullopt, 1000.f, kAllRvc4Backends, {}, false));
}

TEST_CASE("Explicit FPS drops the selection safety margin", "[Depth][ExplicitFps]") {
    // The 768x480 model tops out at 22 FPS on older OS. The margin lets an inferred 22.5 FPS pick it (22 >= 22.5 * 0.9),
    // but an explicitly requested 22.5 FPS must move to a model that can really serve it.
    const std::vector<Depth::Algorithm> neuralOnly = {Depth::Algorithm::NEURAL};
    const auto old = dai::Version(1, 45, 0);
    const auto exact = dai::node::DepthTestAccess::selectWithOs(std::nullopt, 22.5f, neuralOnly, true, old);
    REQUIRE(std::get<DeviceModelZoo>(exact.second) == DeviceModelZoo::NEURAL_DEPTH_576X360);
    const auto margin = dai::node::DepthTestAccess::selectWithOs(std::nullopt, 22.5f, neuralOnly, false, old);
    REQUIRE(std::get<DeviceModelZoo>(margin.second) == DeviceModelZoo::NEURAL_DEPTH_768X480);
}

TEST_CASE("Explicit FPS selection respects the device OS neural capacity", "[Depth][ExplicitFps]") {
    const std::vector<Depth::Algorithm> neuralOnly = {Depth::Algorithm::NEURAL};
    const auto oldOs = dai::node::DepthTestAccess::selectWithOs(std::nullopt, 50.f, neuralOnly, true, dai::Version(1, 45, 0));
    REQUIRE(std::get<DeviceModelZoo>(oldOs.second) == DeviceModelZoo::NEURAL_DEPTH_480X300);
    const auto newOs = dai::node::DepthTestAccess::selectWithOs(std::nullopt, 50.f, neuralOnly, true, dai::Version(1, 47, 0));
    REQUIRE(std::get<DeviceModelZoo>(newOs.second) == DeviceModelZoo::NEURAL_DEPTH_768X480);
}

TEST_CASE("Explicit FPS selection uses the processing size that non-neural backends will receive", "[Depth][ExplicitFps]") {
    const std::vector<Depth::Algorithm> nonNeural = {Depth::Algorithm::NEURAL_ASSISTED_STEREO, Depth::Algorithm::STEREO, Depth::Algorithm::GPU_STEREO};
    const auto largeSensor = std::make_pair(2592u, 1944u);
    // NAS fits the FPS but cannot process this camera size; GPU can serve both constraints.
    const auto selected = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 5.f, nonNeural, {}, true, largeSensor);
    REQUIRE(selected.first == Depth::Algorithm::GPU_STEREO);
    REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::selectOptimized(std::nullopt, 30.f, nonNeural, {}, true, largeSensor),
                        Catch::Matchers::ContainsSubstring("no available algorithm"));
    // A preferred processing size overrides the sensor size for all candidates.
    const auto preferred = dai::node::DepthTestAccess::selectOptimized(std::make_pair(640u, 400u), 30.f, nonNeural, {}, true, largeSensor);
    REQUIRE(preferred.first == Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    // Neural tensors remain independent of the sensor or preview size.
    const auto neural = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 30.f, kAllRvc4Backends, {}, true, largeSensor);
    REQUIRE(neural.first == Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(neural.second) == DeviceModelZoo::NEURAL_DEPTH_1056X660);
}

TEST_CASE("Depth and Camera use the same bounded full-FOV sensor mode", "[Depth][ExplicitFps]") {
    dai::CameraFeatures features;
    features.width = 8000;
    features.height = 6000;
    auto addMode = [&](int32_t width, int32_t height, dai::Rect fov) {
        dai::CameraSensorConfig config;
        config.width = width;
        config.height = height;
        config.type = dai::CameraSensorType::COLOR;
        config.fov = fov;
        features.configs.push_back(config);
    };
    const dai::Rect fullFov(0.f, 0.f, 1.f, 1.f, true);
    addMode(8000, 6000, fullFov);
    addMode(1920, 1440, fullFov);
    addMode(4000, 3000, dai::Rect(0.25f, 0.25f, 0.5f, 0.5f, true));
    const auto processingSize = dai::utility::findDefaultFullResolutionSize(features);
    REQUIRE(processingSize == std::make_pair(1920u, 1440u));
    const auto selected = dai::node::DepthTestAccess::selectOptimized(
        std::nullopt, 5.f, {Depth::Algorithm::NEURAL_ASSISTED_STEREO, Depth::Algorithm::GPU_STEREO}, {}, true, processingSize);
    REQUIRE(selected.first == Depth::Algorithm::GPU_STEREO);
    features.configs.erase(features.configs.begin() + 1);
    const auto unavailable = dai::utility::findDefaultFullResolutionSize(features);
    REQUIRE_FALSE(unavailable);
    // A neural tensor output remains usable when the camera has no default full-FOV mode.
    const auto neural = dai::node::DepthTestAccess::selectOptimized(std::nullopt, 30.f, kAllRvc4Backends, {}, true, unavailable);
    REQUIRE(neural.first == Depth::Algorithm::NEURAL);
}

TEST_CASE("RVC4 stereo preset aliases enforce the equivalent profile FPS limit", "[Depth][ExplicitFps][RabbitRegression]") {
    for(const auto preset : {StereoDepth::PresetMode::FACE,
                             StereoDepth::PresetMode::HIGH_DETAIL,
                             StereoDepth::PresetMode::ROBOTICS,
                             StereoDepth::PresetMode::ACCURACY,
                             StereoDepth::PresetMode::DENSITY}) {
        CAPTURE(preset);
        REQUIRE_NOTHROW(dai::node::DepthTestAccess::validateFps(preset, 30.f));
        REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::validateFps(preset, 31.f), Catch::Matchers::ContainsSubstring("cannot serve the requested 31 FPS"));
    }
    // DEFAULT retains its existing conservative budget; aliases do not change AUTO's preset order.
    REQUIRE_NOTHROW(dai::node::DepthTestAccess::validateFps(StereoDepth::PresetMode::DEFAULT, 16.f));
    REQUIRE_THROWS(dai::node::DepthTestAccess::validateFps(StereoDepth::PresetMode::DEFAULT, 17.f));
}

TEST_CASE("AUTO skips stereo profiles whose width is not aligned", "[Depth][ExplicitFps][RabbitRegression]") {
    const auto size = std::make_pair(648u, 400u);
    const std::vector<Depth::Algorithm> supported = {Depth::Algorithm::STEREO, Depth::Algorithm::GPU_STEREO};
    const auto selected = dai::node::DepthTestAccess::selectOptimized(size, 5.f, supported, {}, true);
    REQUIRE(selected.first == Depth::Algorithm::GPU_STEREO);
    REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::selectOptimized(size, 5.f, kStereoOnly, {}, true),
                        Catch::Matchers::ContainsSubstring("no available algorithm"));
}

TEST_CASE("Processing dimensions are positive before graph construction", "[Depth][ExplicitFps][RabbitRegression]") {
    for(const auto size : {std::make_pair(640u, 0u), std::make_pair(0u, 400u), std::make_pair(0u, 0u)}) {
        CAPTURE(size);
        REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::validateResolution(Depth::Algorithm::STEREO, size),
                            Catch::Matchers::ContainsSubstring("must be positive"));
    }
    REQUIRE_NOTHROW(dai::node::DepthTestAccess::validateResolution(Depth::Algorithm::STEREO, std::make_pair(640u, 400u)));
}

TEST_CASE("Missing bounded sensor modes exclude non-neural default backends", "[Depth][RabbitRegression]") {
    for(const auto algorithm : {Depth::Algorithm::STEREO, Depth::Algorithm::GPU_STEREO, Depth::Algorithm::NEURAL_ASSISTED_STEREO}) {
        REQUIRE_THROWS_WITH(dai::node::DepthTestAccess::validateResolution(algorithm, std::nullopt),
                            Catch::Matchers::ContainsSubstring("no valid full resolution configuration"));
    }
    REQUIRE_NOTHROW(dai::node::DepthTestAccess::validateResolution(Depth::Algorithm::NEURAL, std::nullopt));
    REQUIRE_NOTHROW(dai::node::DepthTestAccess::validateResolution(Depth::Algorithm::TOF, std::nullopt));
}
