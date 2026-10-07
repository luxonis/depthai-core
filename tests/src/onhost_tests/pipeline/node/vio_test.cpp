#include "depthai/pipeline/node/VIO.hpp"

#include <catch2/catch_test_macros.hpp>
#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

TEST_CASE("VIO configuration is validated and survives firmware serialization") {
    const auto vio = dai::node::VIO::create();
    REQUIRE_FALSE(vio->runOnHost());
    REQUIRE_THROWS_AS(vio->setImuUpdateRate(0), std::invalid_argument);
    REQUIRE_THROWS_AS(vio->setImuUpdateRate(-1), std::invalid_argument);
    vio->setImuUpdateRate(400).setUseSpecTranslation(false);
    dai::VIOProperties decoded;
    dai::utility::deserialize(dai::utility::serialize(vio->properties), decoded);
    CHECK(decoded.imuFrequency == 400);
    CHECK_FALSE(decoded.useSpecTranslation);
    // A device-only node must reject a host-only pipeline, not run a silent fallback.
    REQUIRE_THROWS_AS(vio->buildStage1(), std::runtime_error);
}
