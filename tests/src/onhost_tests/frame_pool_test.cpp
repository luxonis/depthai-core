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
        for(auto type : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON}) {
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
    const auto fps = GENERATE(0.0, -1.0, 30.0, 60.0, 120.0, 1000.0);
    const auto expected = fps <= 30 ? 4u : fps == 60 ? 6u : fps == 120 ? 12u : 30u;
    const auto resolved = resolveFramePool({}, 4096, fps);
    REQUIRE(resolved.outputFrameSize == 4096);
    REQUIRE(resolved.numFramesPool == expected);
    REQUIRE(resolved.maxPoolSize == 4096 * expected);
    REQUIRE(resolveFramePool({}, 4096, std::numeric_limits<double>::infinity()).numFramesPool == 4);
    REQUIRE(resolveFramePool({}, 4096, std::numeric_limits<double>::quiet_NaN()).numFramesPool == 4);
    REQUIRE(resolveFramePool({}, 32 * 1024 * 1024, 120).numFramesPool == 2);
}

TEST_CASE("All combinations of automatic and explicit frame pool fields resolve consistently", "[frame-pool]") {
    const auto frameSize = GENERATE(0u, 2048u);
    const auto count = GENERATE(0u, 6u);
    const auto maxSize = GENERATE(0u, 16384u);
    FramePoolProperties request{frameSize, count, maxSize};
    const auto resolved = resolveFramePool(request, 1024, 60);
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
    REQUIRE(resolveFramePool({0, 0, 1024}, 1024).numFramesPool == 1);
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
    };
    check(std::make_shared<dai::node::ImageManip>());
    check(std::make_shared<dai::node::StereoDepth>());
    check(std::make_shared<dai::node::ImageAlign>());
    check(std::make_shared<dai::node::Align>());
}
