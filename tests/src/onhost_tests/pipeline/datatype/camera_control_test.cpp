#include <catch2/catch_all.hpp>
#include <stdexcept>
#include <tuple>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace {

std::tuple<int, int, int, int> asTuple(const dai::CameraControl::RegionParams& region) {
    return {region.x, region.y, region.width, region.height};
}

// IMX586 on RVC4: an 8000x6000 source binned to a 4000x3000 output.
dai::ImgTransformation binned() {
    dai::ImgTransformation transformation(8000, 6000);
    transformation.addScale(0.5f, 0.5f);
    return transformation;
}

// IMX586 on RVC4: the 1920x1080 sensor mode sees 7680x4320 at (160, 840) of the source, scaled by 1/4.
dai::ImgTransformation cropped() {
    dai::ImgTransformation transformation(8000, 6000);
    transformation.addCrop(160, 840, 7680, 4320);
    transformation.addScale(0.25f, 0.25f);
    return transformation;
}

}  // namespace

TEST_CASE("Region from a frame maps to its source frame", "[CameraControl][Region]") {
    dai::CameraControl control;

    SECTION("same size") {
        control.setAutoFocusRegion(dai::Rect(10, 20, 30, 40, false), dai::ImgTransformation(4000, 3000));
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(10, 20, 30, 40));
    }
    SECTION("binned") {
        control.setAutoFocusRegion(dai::Rect(2179, 2096, 126, 226, false), binned());
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(4358, 4192, 252, 452));
    }
    SECTION("cropped") {
        control.setAutoExposureRegion(dai::Rect(100, 50, 40, 20, false), cropped());
        REQUIRE(asTuple(control.aeRegion) == std::make_tuple(560, 1040, 160, 80));
    }
    SECTION("normalized") {
        control.setAutoExposureRegion(dai::Rect(0.5f, 0.5f, 0.25f, 0.25f, true), cropped());
        REQUIRE(asTuple(control.aeRegion) == std::make_tuple(4000, 3000, 1920, 1080));
    }
    SECTION("normalized, between output pixels") {
        // An 8000x6000 source scaled to 640x480: a normalized box maps as the same box in fractional pixels does.
        dai::ImgTransformation transformation(8000, 6000);
        transformation.addScale(0.08f, 0.08f);
        control.setAutoFocusRegion(dai::Rect(0.1005f, 0.1005f, 0.1f, 0.1f, true), transformation);
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(804, 603, 800, 600));
        control.setAutoFocusRegion(dai::Rect(64.32f, 48.24f, 64.0f, 48.0f, false), transformation);
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(804, 603, 800, 600));
    }
    SECTION("normalized, narrower than half an output pixel") {
        // 0.0005 of 640 px is 0.32 px, 4 px of the source.
        dai::ImgTransformation transformation(8000, 6000);
        transformation.addScale(0.08f, 0.08f);
        control.setAutoExposureRegion(dai::Rect(0.5f, 0.5f, 0.0005f, 0.1f, true), transformation);
        REQUIRE(asTuple(control.aeRegion) == std::make_tuple(4000, 3000, 4, 600));
    }
    SECTION("through a further resize") {
        // A 640x360 NN input made from the cropped 1920x1080 output.
        auto transformation = cropped();
        transformation.addScale(1.0f / 3, 1.0f / 3);
        control.setAutoFocusRegion(dai::Rect(320, 180, 64, 36, false), transformation);
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(4000, 3000, 768, 432));
    }
    SECTION("fractional edges widen to whole source pixels") {
        dai::ImgTransformation transformation(8000, 6000);
        transformation.addScale(0.3f, 0.3f);  // 2400x1800
        control.setAutoFocusRegion(dai::Rect(1, 1, 1, 1, false), transformation);
        REQUIRE(asTuple(control.afRegion) == std::make_tuple(3, 3, 4, 4));
    }
    SECTION("clamped to the source") {
        control.setAutoExposureRegion(dai::Rect(-10, 2990, 30, 30, false), binned());
        REQUIRE(asTuple(control.aeRegion) == std::make_tuple(0, 5980, 40, 20));
    }
    SECTION("priority as with pixel coordinates") {
        control.setAutoFocusRegion(dai::Rect(10, 20, 30, 40, false), binned());
        REQUIRE(control.afRegion.priority == 1);
    }
}

TEST_CASE("Region from a frame rejects what cannot be mapped", "[CameraControl][Region]") {
    dai::CameraControl control;
    REQUIRE_THROWS_AS(control.setAutoFocusRegion(dai::Rect(5000, 10, 20, 20, false), binned()), std::invalid_argument);
    REQUIRE_THROWS_AS(control.setAutoExposureRegion(dai::Rect(10, 10, 20, 20, false), dai::ImgTransformation()), std::invalid_argument);
    // Empty boxes, including a negative width, which would otherwise flip into a box left of x.
    REQUIRE_THROWS_AS(control.setAutoFocusRegion(dai::Rect(10, 20, -5, 10, false), binned()), std::invalid_argument);
    REQUIRE_THROWS_AS(control.setAutoFocusRegion(dai::Rect(10, 20, 0, 10, false), binned()), std::invalid_argument);
    REQUIRE_THROWS_AS(control.setAutoExposureRegion(dai::Rect(0.1f, 0.1f, 0.2f, 0.0f, true), binned()), std::invalid_argument);
}
