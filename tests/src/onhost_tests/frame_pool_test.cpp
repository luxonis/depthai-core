#include <catch2/catch_all.hpp>
#include <limits>

#include "depthai/pipeline/node/Align.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/properties/AlignProperties.hpp"
#include "depthai/properties/ImageAlignProperties.hpp"
#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"
#include "depthai/utility/FramePool.hpp"

using dai::FramePoolProperties;
using dai::utility::resolveFramePool;

TEST_CASE("Frame pool properties default to automatic and roundtrip", "[frame-pool]") {
    const auto check = [](auto properties) {
        REQUIRE(properties.pool.outputFrameSize == 0);
        REQUIRE(properties.pool.numFramesPool == 0);
        REQUIRE(properties.pool.maxPoolSize == 0);
        properties.pool = {12345, 6, 99999};
        for(auto type : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            const auto bytes = dai::utility::serialize(properties, type);
            decltype(properties) decoded;
            dai::utility::deserialize(bytes, decoded, type);
            REQUIRE(decoded.pool.outputFrameSize == 12345);
            REQUIRE(decoded.pool.numFramesPool == 6);
            REQUIRE(decoded.pool.maxPoolSize == 99999);
        }
    };
    check(dai::ImageManipProperties{});
    check(dai::StereoDepthProperties{});
    check(dai::ImageAlignProperties{});
    check(dai::AlignProperties{});
}

TEST_CASE("Automatic frame pools use FPS and required padded frame size", "[frame-pool]") {
    const auto fps = GENERATE(0.0, -1.0, 10.0, 20.0, 30.0, 60.0, 120.0, 1000.0);
    const auto expected = fps == 10 ? 2u : fps == 20 ? 4u : fps <= 30 ? 6u : fps == 60 ? 12u : fps == 120 ? 24u : 30u;
    const auto resolved = resolveFramePool({}, 4096, fps);
    REQUIRE(resolved.outputFrameSize == 4096);
    REQUIRE(resolved.numFramesPool == expected);
    REQUIRE(resolved.maxPoolSize == 4096 * expected);
    REQUIRE(resolveFramePool({}, 4096, std::numeric_limits<double>::infinity()).numFramesPool == 6);
    REQUIRE(resolveFramePool({}, 4096, std::numeric_limits<double>::quiet_NaN()).numFramesPool == 6);
    REQUIRE(resolveFramePool({}, 32 * 1024 * 1024, 120).numFramesPool == 6);
}

TEST_CASE("All combinations of automatic and explicit frame pool fields resolve consistently", "[frame-pool]") {
    const auto frameSize = GENERATE(0u, 2048u);
    const auto count = GENERATE(0u, 6u);
    const auto maxSize = GENERATE(0u, 16384u);
    FramePoolProperties request{frameSize, count, maxSize};
    const auto resolved = resolveFramePool(request, 1024, 30);
    REQUIRE(resolved.outputFrameSize == (frameSize ? frameSize : 1024));
    REQUIRE(resolved.numFramesPool == 6);
    REQUIRE(resolved.maxPoolSize >= resolved.outputFrameSize * resolved.numFramesPool);
    if(maxSize) REQUIRE(resolved.maxPoolSize == maxSize);
    REQUIRE(request.outputFrameSize == frameSize);  // Resolving must preserve AUTO for subsequent frame-size changes.
    REQUIRE(request.numFramesPool == count);
    REQUIRE(request.maxPoolSize == maxSize);
}

TEST_CASE("Hard pool budgets reduce automatic counts and reject impossible requests", "[frame-pool]") {
    REQUIRE(resolveFramePool({0, 0, 3072}, 1024, 120).numFramesPool == 3);
    REQUIRE(resolveFramePool({0, 0, 2048}, 1024).numFramesPool == 2);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 1024}, 1024), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 1, 0}, 1024), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 1, 0}, 1024, 30, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 1023}, 1024), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 4, 4095}, 1024), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({1023, 0, 0}, 1024), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({}, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 3, 0}, 1024, 30, 4), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 3072}, 1024, 30, 4), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({std::numeric_limits<uint64_t>::max(), 2, 0}, 1), std::overflow_error);
    // Explicit counts override only the soft automatic budget.
    REQUIRE(resolveFramePool({0, 8, 0}, 32 * 1024 * 1024).numFramesPool == 8);
}

TEST_CASE("Automatic frame pool resolution follows dynamic output size", "[frame-pool]") {
    const FramePoolProperties automatic;
    REQUIRE(resolveFramePool(automatic, 1024).outputFrameSize == 1024);
    REQUIRE(resolveFramePool(automatic, 8192).outputFrameSize == 8192);
    REQUIRE(resolveFramePool({16384, 0, 0}, 8192).outputFrameSize == 16384);
    REQUIRE_THROWS_AS(resolveFramePool({4096, 0, 0}, 8192), std::invalid_argument);
}

TEST_CASE("Node pool setters update the shared configuration", "[frame-pool]") {
    const auto check = [](auto node) {
        node->setPoolProperties({8192, 6, 65536});
        REQUIRE(node->properties.pool.outputFrameSize == 8192);
        REQUIRE(node->properties.pool.numFramesPool == 6);
        REQUIRE(node->properties.pool.maxPoolSize == 65536);
        node->setNumFramesPool(8);
        REQUIRE(node->properties.pool.numFramesPool == 8);
        REQUIRE(node->properties.pool.outputFrameSize == 8192);
        REQUIRE(node->properties.pool.maxPoolSize == 65536);
        node->setNumFramesPool(0);
        REQUIRE(node->properties.pool.numFramesPool == 0);
        REQUIRE_THROWS_AS(node->setNumFramesPool(-1), std::invalid_argument);
        REQUIRE_THROWS_AS(node->setNumFramesPool(1), std::invalid_argument);
        REQUIRE_THROWS_AS(node->setPoolProperties({8192, 1, 8192}), std::invalid_argument);
        REQUIRE(node->properties.pool.numFramesPool == 0);
        node->setNumFramesPool(2);
        REQUIRE(node->properties.pool.numFramesPool == 2);
    };
    check(std::make_shared<dai::node::ImageManip>());
    check(std::make_shared<dai::node::StereoDepth>());
    check(std::make_shared<dai::node::ImageAlign>());
    check(std::make_shared<dai::node::Align>());
}

TEST_CASE("Automatic frame pools fit at least two maximum-resolution buffers", "[frame-pool]") {
    // 8K UHD RGB, full IMX586 RGB, and FP16 RGB with backend padding.
    const auto size = GENERATE(uint64_t(7680) * 4320 * 3, uint64_t(8000) * 6000 * 3, uint64_t(8000) * 6000 * 6 + 4096);
    const auto resolved = resolveFramePool({}, size, 30);
    REQUIRE(resolved.numFramesPool >= 2);
    REQUIRE(resolved.maxPoolSize >= size * 2);
    const auto highFps = resolveFramePool({}, size, 120);
    REQUIRE(highFps.numFramesPool >= 2);
    REQUIRE(highFps.maxPoolSize >= size * 2);
    REQUIRE(resolveFramePool({0, 0, size * 2}, size, 120).numFramesPool == 2);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, size * 2 - 1}, size, 120), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 1, 0}, size), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({std::numeric_limits<uint64_t>::max() / 2 + 1, 0, 0}, 1), std::overflow_error);
}

namespace {
template <typename T>
struct PoolPropertiesProbe : T {
    using T::getProperties;
};
}  // namespace

TEST_CASE("Direct edits to pool counts are validated before serialization", "[frame-pool]") {
    const auto check = [](auto node) {
        node->properties.pool.numFramesPool = 1;
        REQUIRE_THROWS_AS(node->getProperties(), std::invalid_argument);
        node->properties.pool.numFramesPool = 2;
        REQUIRE_NOTHROW(node->getProperties());
    };
    check(std::make_shared<PoolPropertiesProbe<dai::node::ImageManip>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::StereoDepth>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::ImageAlign>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::Align>>());
}

TEST_CASE("Automatic budgets expand only to the algorithm minimum", "[frame-pool]") {
    constexpr uint64_t MiB = 1024 * 1024;
    const auto large = resolveFramePool({}, 200 * MiB, 120);
    REQUIRE(large.numFramesPool == 2);
    REQUIRE(large.maxPoolSize == 400 * MiB);
    const auto algorithm = resolveFramePool({}, 200 * MiB, 120, 4);
    REQUIRE(algorithm.numFramesPool == 4);
    REQUIRE(algorithm.maxPoolSize == 800 * MiB);
    const auto lowFpsAlgorithm = resolveFramePool({}, 4096, 10, 4);
    REQUIRE(lowFpsAlgorithm.numFramesPool == 4);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 600 * MiB}, 200 * MiB, 120, 4), std::invalid_argument);
}

TEST_CASE("Default policy resolves full-resolution RGB output to two initial frames", "[frame-pool]") {
    const auto size = GENERATE(uint64_t(7680) * 4320 * 3, uint64_t(8000) * 6000 * 3);
    const auto padding = GENERATE(uint64_t(0), uint64_t(4096));
    const auto fps = GENERATE(30.0, 60.0, 120.0);
    const auto resolved = resolveFramePool({}, size + padding, fps);
    REQUIRE(resolved.numFramesPool == 2);
    REQUIRE(resolved.maxPoolSize == (size + padding) * 2);
}

TEST_CASE("Legacy pool settings are reconciled before node serialization", "[frame-pool]") {
    const auto check = [](auto node) {
        node->properties.numFramesPool = 7;
        REQUIRE(node->getProperties().pool.numFramesPool == 7);
        node->properties.pool.numFramesPool = 9;
        REQUIRE(node->getProperties().pool.numFramesPool == 9);
        node->setPoolProperties({});
        REQUIRE(node->getProperties().pool.numFramesPool == 0);
        node->properties.numFramesPool = -1;
        REQUIRE_THROWS_AS(node->getProperties(), std::invalid_argument);
    };
    check(std::make_shared<PoolPropertiesProbe<dai::node::ImageManip>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::StereoDepth>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::ImageAlign>>());
    check(std::make_shared<PoolPropertiesProbe<dai::node::Align>>());
    PoolPropertiesProbe<dai::node::ImageManip> manip;
    manip.properties.outputFrameSize = 8192;
    manip.properties.maxPoolSize = 65536;
    REQUIRE(manip.getProperties().pool.outputFrameSize == 8192);
    REQUIRE(manip.getProperties().pool.maxPoolSize == 65536);
    manip.setPoolProperties({});
    REQUIRE(manip.getProperties().pool.outputFrameSize == 0);
    REQUIRE(manip.getProperties().pool.maxPoolSize == 0);
}

TEST_CASE("Automatic initial counts honor platform defaults and algorithm minimums", "[frame-pool]") {
    const auto initial = GENERATE(3u, 4u);
    const auto budget = dai::utility::DEFAULT_FRAME_POOL_BUDGET;
    const auto lowFps = resolveFramePool({}, 4096, 10, 2, budget, initial);
    REQUIRE(lowFps.numFramesPool == initial);
    REQUIRE(lowFps.maxPoolSize == 4096 * initial);
    REQUIRE(resolveFramePool({}, 4096, 60, 2, budget, initial).numFramesPool == 12);
    REQUIRE(resolveFramePool({}, 4096, 10, 4, budget, initial).numFramesPool == 4);
    REQUIRE(resolveFramePool({0, 2, 0}, 4096, 120, 2, budget, initial).numFramesPool == 2);
    REQUIRE_THROWS_AS(resolveFramePool({0, 3, 0}, 4096, 10, 4, budget, initial), std::invalid_argument);
}

TEST_CASE("Explicit budgets constrain platform initial counts without relaxing algorithm minimums", "[frame-pool]") {
    const auto initial = GENERATE(3u, 4u);
    const auto budget = dai::utility::DEFAULT_FRAME_POOL_BUDGET;
    REQUIRE(resolveFramePool({0, 0, 8192}, 4096, 10, 2, budget, initial).numFramesPool == 2);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 4096}, 4096, 10, 2, budget, initial), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 0, 12288}, 4096, 10, 4, budget, initial), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({0, 4, 12288}, 4096, 10, 2, budget, initial), std::invalid_argument);
    REQUIRE_THROWS_AS(resolveFramePool({2048, 0, 0}, 4096, 10, 2, budget, initial), std::invalid_argument);
}

TEST_CASE("Pool growth policy uses the supplied platform budget and preserves fixed counts", "[frame-pool]") {
    using dai::utility::framePoolGrowthBudget;
    constexpr uint64_t platformBudget = 16384;
    const auto size = GENERATE(uint64_t(1024), uint64_t(8192));
    const FramePoolProperties automatic;
    const auto resolved = resolveFramePool(automatic, size, 10, 2, platformBudget, 4);
    REQUIRE(resolved.numFramesPool == 4);
    REQUIRE(framePoolGrowthBudget(automatic, resolved, platformBudget) == (size == 1024 ? platformBudget : 32768));

    const FramePoolProperties limited{0, 0, size * 2};
    const auto constrained = resolveFramePool(limited, size, 120, 2, platformBudget, 4);
    REQUIRE(constrained.numFramesPool == 2);
    REQUIRE(framePoolGrowthBudget(limited, constrained, platformBudget) == limited.maxPoolSize);

    const FramePoolProperties fixed{0, 8, 0};
    const auto explicitCount = resolveFramePool(fixed, size, 120, 2, platformBudget, 4);
    REQUIRE(explicitCount.numFramesPool == 8);
    REQUIRE(framePoolGrowthBudget(fixed, explicitCount, platformBudget) == 0);
}

TEST_CASE("Platform initial floors expand soft budgets with checked arithmetic", "[frame-pool]") {
    constexpr uint64_t MiB = 1024 * 1024;
    const auto initial = GENERATE(3u, 4u);
    const auto resolved = resolveFramePool({}, 200 * MiB, 120, 2, 32 * MiB, initial);
    REQUIRE(resolved.numFramesPool == initial);
    REQUIRE(resolved.maxPoolSize == 200 * MiB * initial);
    REQUIRE(dai::utility::framePoolGrowthBudget({}, resolved, 32 * MiB) == resolved.maxPoolSize);
    REQUIRE_THROWS_AS(resolveFramePool({}, std::numeric_limits<uint64_t>::max() / initial + 1, 10, 2, 32 * MiB, initial), std::overflow_error);
}
