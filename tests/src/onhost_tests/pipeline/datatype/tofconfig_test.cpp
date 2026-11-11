#include "depthai/pipeline/datatype/ToFConfig.hpp"

#include <catch2/catch_test_macros.hpp>

#include "depthai/properties/ToFProperties.hpp"
#include "depthai/utility/Serialization.hpp"

TEST_CASE("Intermediate ToF depth selection reaches initial and runtime configurations", "[ToFConfig]") {
    REQUIRE(dai::ToFConfig{}.enableIntermediateDepth);
    for(bool enable : {false, true}) {
        dai::ToFProperties properties;
        properties.initialConfig.enableIntermediateDepth = enable;
        const auto initialBytes = dai::utility::serialize(properties);
        dai::ToFProperties restored;
        REQUIRE(dai::utility::deserialize(initialBytes, restored));
        REQUIRE(restored.initialConfig.enableIntermediateDepth == enable);

        std::vector<std::uint8_t> metadata;
        dai::DatatypeEnum datatype;
        properties.initialConfig.serialize(metadata, datatype);
        REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
        dai::ToFConfig runtimeConfig;
        REQUIRE(dai::utility::deserialize(metadata, runtimeConfig));
        REQUIRE(runtimeConfig.enableIntermediateDepth == enable);
    }
}
