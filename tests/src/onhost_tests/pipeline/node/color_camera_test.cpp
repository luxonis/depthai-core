#include <catch2/catch_all.hpp>
#include <limits>
#include <stdexcept>

#include "depthai/pipeline/node/ColorCamera.hpp"

TEST_CASE("ColorCamera scaled size validates arithmetic") {
    dai::node::ColorCamera camera;
    REQUIRE(camera.getScaledSize(640, 1, 2) == 320);
    REQUIRE(camera.getScaledSize(641, 1, 2) == 321);
    REQUIRE_THROWS_AS(camera.getScaledSize(640, 1, 0), std::invalid_argument);

    const auto max = std::numeric_limits<int>::max();
    const auto min = std::numeric_limits<int>::min();
    REQUIRE(camera.getScaledSize(max, 2, 2) == max);
    REQUIRE(camera.getScaledSize(max, max, max) == max);
    REQUIRE_THROWS_AS(camera.getScaledSize(max, 2, 1), std::overflow_error);
    REQUIRE_THROWS_AS(camera.getScaledSize(min, 2, 1), std::overflow_error);
}
