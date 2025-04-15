#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <depthai/modelzoo/Zoo.hpp>
#include <sstream>
#include <string>

using dai::SlugComponents;
TEST_CASE("SlugComponents::merge", "[SlugComponents]") {
    SECTION("Merge with all components") {
        SlugComponents components{"teamName", "modelSlug", "variantSlug", "variantHash"};
        REQUIRE(components.merge() == "teamName/modelSlug:variantSlug:variantHash");
    }

    SECTION("Merge without teamName") {
        SlugComponents components{"", "modelSlug", "variantSlug", "variantHash"};
        REQUIRE(components.merge() == "modelSlug:variantSlug:variantHash");
    }

    SECTION("Merge without modelVariantSlug") {
        SlugComponents components{"teamName", "modelSlug", "", "variantHash"};
        REQUIRE(components.merge() == "teamName/modelSlug:variantHash");
    }

    SECTION("Merge without modelRef") {
        SlugComponents components{"teamName", "modelSlug", "variantSlug", ""};
        REQUIRE(components.merge() == "teamName/modelSlug:variantSlug");
    }

    SECTION("Merge with only modelSlug") {
        SlugComponents components{"", "modelSlug", "", ""};
        REQUIRE(components.merge() == "modelSlug");
    }
}

TEST_CASE("SlugComponents::split", "[SlugComponents]") {
    SECTION("Split with all components") {
        SlugComponents components = SlugComponents::split("teamName/modelSlug:variantSlug:variantHash");
        REQUIRE(components.teamName == "teamName");
        REQUIRE(components.modelSlug == "modelSlug");
        REQUIRE(components.modelVariantSlug == "variantSlug");
        REQUIRE(components.modelRef == "variantHash");
    }

    SECTION("Split without teamName") {
        SlugComponents components = SlugComponents::split("modelSlug:variantSlug:variantHash");
        REQUIRE(components.teamName.empty());
        REQUIRE(components.modelSlug == "modelSlug");
        REQUIRE(components.modelVariantSlug == "variantSlug");
        REQUIRE(components.modelRef == "variantHash");
    }

    SECTION("Split without modelRef") {
        SlugComponents components = SlugComponents::split("teamName/modelSlug:variantSlug");
        REQUIRE(components.teamName == "teamName");
        REQUIRE(components.modelSlug == "modelSlug");
        REQUIRE(components.modelVariantSlug == "variantSlug");
        REQUIRE(components.modelRef.empty());
    }

    SECTION("Split with only modelSlug") {
        SlugComponents components = SlugComponents::split("modelSlug");
        REQUIRE(components.teamName.empty());
        REQUIRE(components.modelSlug == "modelSlug");
        REQUIRE(components.modelVariantSlug.empty());
        REQUIRE(components.modelRef.empty());
    }
}
