#include <catch2/catch_all.hpp>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

TEST_CASE("DatatypeEnum hierarchy is complete for all enum values", "[datatype][hierarchy]") {
    using dai::DatatypeEnum;
    const auto lastValue = static_cast<int>(DatatypeEnum::PacketizedData);

    for(int i = 0; i <= lastValue; ++i) {
        const auto dt = static_cast<DatatypeEnum>(i);

        INFO("DatatypeEnum raw value: " << i);

        // Ensures every enum value has an entry as a parent in hierarchy map.
        REQUIRE_NOTHROW((void)dai::isDatatypeSubclassOf(dt, DatatypeEnum::ADatatype));

        // Every datatype except ADatatype itself must be a descendant of ADatatype.
        if(dt != DatatypeEnum::ADatatype) {
            REQUIRE(dai::isDatatypeSubclassOf(DatatypeEnum::ADatatype, dt));
        }

        // In current design, all concrete datatypes are Buffer descendants.
        if(dt != DatatypeEnum::ADatatype && dt != DatatypeEnum::Buffer) {
            REQUIRE(dai::isDatatypeSubclassOf(DatatypeEnum::Buffer, dt));
        }
    }
}
