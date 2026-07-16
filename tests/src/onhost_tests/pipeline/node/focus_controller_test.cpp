#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/node/host/FocusController.hpp>
#include <vector>

using dai::node::FocusController;

namespace {

constexpr int kW = 1280;
constexpr int kH = 800;

// Every crop must stay fully inside the frame, regardless of where the detection was.
void requireCropInsideFrame(const FocusController::Crop& c, int width, int height) {
    REQUIRE(c.x >= 0);
    REQUIRE(c.y >= 0);
    REQUIRE(c.w > 0);
    REQUIRE(c.h > 0);
    REQUIRE(c.x + c.w <= width);
    REQUIRE(c.y + c.h <= height);
}

std::array<float, 4> box(float xmin, float ymin, float xmax, float ymax) {
    return {xmin, ymin, xmax, ymax};
}

}  // namespace

TEST_CASE("FocusController::computeCrops: multiple regions map to one crop each", "[FocusController]") {
    const std::vector<std::array<float, 4>> boxes = {
        box(0.35f, 0.30f, 0.65f, 0.70f),
        box(0.10f, 0.55f, 0.30f, 0.80f),
        box(0.70f, 0.10f, 0.90f, 0.35f),
    };

    const auto crops = FocusController::computeCrops(kW, kH, boxes);
    REQUIRE(crops.size() == boxes.size());

    for(std::size_t i = 0; i < crops.size(); ++i) {
        requireCropInsideFrame(crops[i], kW, kH);
        // The detection box (used to place the focused output back) matches the input box in pixels.
        REQUIRE(crops[i].detX == Catch::Approx(boxes[i][0] * kW));
        REQUIRE(crops[i].detY == Catch::Approx(boxes[i][1] * kH));
        REQUIRE(crops[i].detW == Catch::Approx((boxes[i][2] - boxes[i][0]) * kW));
        REQUIRE(crops[i].detH == Catch::Approx((boxes[i][3] - boxes[i][1]) * kH));
        // The backend crop is padded horizontally for disparity, so it is wider than the detection.
        REQUIRE(static_cast<float>(crops[i].w) >= crops[i].detW);
    }
}

TEST_CASE("FocusController::computeCrops: crop count follows the number of valid regions", "[FocusController]") {
    SECTION("no regions") {
        REQUIRE(FocusController::computeCrops(kW, kH, {}).empty());
    }
    SECTION("single region") {
        REQUIRE(FocusController::computeCrops(kW, kH, {box(0.4f, 0.4f, 0.6f, 0.6f)}).size() == 1);
    }
    SECTION("many regions") {
        const std::vector<std::array<float, 4>> boxes = {
            box(0.05f, 0.05f, 0.15f, 0.15f),
            box(0.25f, 0.25f, 0.35f, 0.35f),
            box(0.45f, 0.45f, 0.55f, 0.55f),
            box(0.65f, 0.65f, 0.75f, 0.75f),
            box(0.80f, 0.80f, 0.95f, 0.95f),
        };
        REQUIRE(FocusController::computeCrops(kW, kH, boxes).size() == boxes.size());
    }
    SECTION("degenerate and fully out-of-frame regions are dropped, valid ones kept") {
        const std::vector<std::array<float, 4>> boxes = {
            box(0.30f, 0.30f, 0.50f, 0.50f),  // valid
            box(0.50f, 0.50f, 0.50f, 0.70f),  // zero width -> dropped
            box(0.40f, 0.60f, 0.60f, 0.60f),  // zero height -> dropped
            box(0.70f, 0.50f, 0.40f, 0.60f),  // xmax < xmin -> dropped
            box(1.30f, 0.40f, 1.50f, 0.60f),  // entirely right of frame even after padding -> dropped
            box(0.20f, 0.20f, 0.40f, 0.45f),  // valid
        };
        REQUIRE(FocusController::computeCrops(kW, kH, boxes).size() == 2);
    }
}

TEST_CASE("FocusController::computeCrops: regions touching the frame edges stay clamped", "[FocusController]") {
    SECTION("left edge") {
        const auto crops = FocusController::computeCrops(kW, kH, {box(0.0f, 0.40f, 0.20f, 0.60f)});
        REQUIRE(crops.size() == 1);
        requireCropInsideFrame(crops[0], kW, kH);
        REQUIRE(crops[0].x == 0);
    }
    SECTION("right edge") {
        const auto crops = FocusController::computeCrops(kW, kH, {box(0.80f, 0.40f, 1.0f, 0.60f)});
        REQUIRE(crops.size() == 1);
        requireCropInsideFrame(crops[0], kW, kH);
        REQUIRE(crops[0].x + crops[0].w == kW);
    }
    SECTION("top edge") {
        const auto crops = FocusController::computeCrops(kW, kH, {box(0.40f, 0.0f, 0.60f, 0.20f)});
        REQUIRE(crops.size() == 1);
        requireCropInsideFrame(crops[0], kW, kH);
        REQUIRE(crops[0].y == 0);
    }
    SECTION("bottom edge") {
        const auto crops = FocusController::computeCrops(kW, kH, {box(0.40f, 0.80f, 0.60f, 1.0f)});
        REQUIRE(crops.size() == 1);
        requireCropInsideFrame(crops[0], kW, kH);
        REQUIRE(crops[0].y + crops[0].h == kH);
    }
    SECTION("full frame covers the whole image") {
        const auto crops = FocusController::computeCrops(kW, kH, {box(0.0f, 0.0f, 1.0f, 1.0f)});
        REQUIRE(crops.size() == 1);
        REQUIRE(crops[0].x == 0);
        REQUIRE(crops[0].y == 0);
        REQUIRE(crops[0].w == kW);
        REQUIRE(crops[0].h == kH);
    }
    SECTION("all four corners at once stay inside the frame") {
        const std::vector<std::array<float, 4>> boxes = {
            box(0.0f, 0.0f, 0.15f, 0.15f),
            box(0.85f, 0.0f, 1.0f, 0.15f),
            box(0.0f, 0.85f, 0.15f, 1.0f),
            box(0.85f, 0.85f, 1.0f, 1.0f),
        };
        const auto crops = FocusController::computeCrops(kW, kH, boxes);
        REQUIRE(crops.size() == boxes.size());
        for(const auto& c : crops) {
            requireCropInsideFrame(c, kW, kH);
        }
    }
}
