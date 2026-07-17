#include <algorithm>
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

// The reassembly region must index both the crop mat (w x h) and the full frame without overflow.
void requireRegionFits(const FocusController::CopyRegion& r, const FocusController::Crop& c, int width, int height) {
    REQUIRE(r.srcX >= 0);
    REQUIRE(r.srcY >= 0);
    REQUIRE(r.w > 0);
    REQUIRE(r.h > 0);
    REQUIRE(r.srcX + r.w <= c.w);
    REQUIRE(r.srcY + r.h <= c.h);
    REQUIRE(r.dstX >= 0);
    REQUIRE(r.dstY >= 0);
    REQUIRE(r.dstX + r.w <= width);
    REQUIRE(r.dstY + r.h <= height);
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

TEST_CASE("FocusController::computeCopyRegion: reassembly region never overflows the crop or frame", "[FocusController]") {
    // Sweep fractional boxes: detections from a detector rarely land on integer pixels, and
    // floor(detY)+ceil(detH) (the crop height) can differ from ceil(detY+detH)-floor(detY) (the
    // reassembly height) by a pixel. The region must be clamped so neither cv::Mat ROI overflows.
    for(int xi = 0; xi < 20; ++xi) {
        for(int yi = 0; yi < 20; ++yi) {
            const float xmin = 0.013f + xi * 0.047f;
            const float ymin = 0.017f + yi * 0.043f;
            const float xmax = std::min(0.999f, xmin + 0.091f);
            const float ymax = std::min(0.999f, ymin + 0.073f);
            const auto crops = FocusController::computeCrops(kW, kH, {box(xmin, ymin, xmax, ymax)});
            if(crops.empty()) {
                continue;
            }
            const auto region = FocusController::computeCopyRegion(crops[0], kW, kH);
            REQUIRE(region.valid);
            requireRegionFits(region, crops[0], kW, kH);
        }
    }
}

TEST_CASE("FocusController::computeCopyRegion: fractional height rounding is clamped to the crop", "[FocusController]") {
    // Detection box whose fractional offset + fractional height crosses an integer boundary:
    // ceil(detY + detH) - floor(detY) = 7 but the crop is only ceil(detH) = 6 rows tall.
    FocusController::Crop crop{100, 10, 40, 6, 100.4f, 10.6f, 30.0f, 5.6f};
    const auto region = FocusController::computeCopyRegion(crop, kW, kH);
    REQUIRE(region.valid);
    requireRegionFits(region, crop, kW, kH);
    REQUIRE(region.srcY == 0);
    REQUIRE(region.h == crop.h);
}

TEST_CASE("FocusController::computeCopyRegion: edge-touching detection stays inside both buffers", "[FocusController]") {
    const std::vector<std::array<float, 4>> boxes = {
        box(0.0f, 0.0f, 0.12f, 0.12f),     // top-left corner
        box(0.90f, 0.0f, 1.0f, 0.13f),     // top-right corner
        box(0.0f, 0.88f, 0.11f, 1.0f),     // bottom-left corner
        box(0.88f, 0.87f, 1.0f, 1.0f),     // bottom-right corner
        box(0.0f, 0.0f, 1.0f, 1.0f),       // whole frame
    };
    const auto crops = FocusController::computeCrops(kW, kH, boxes);
    REQUIRE(crops.size() == boxes.size());
    for(const auto& c : crops) {
        const auto region = FocusController::computeCopyRegion(c, kW, kH);
        REQUIRE(region.valid);
        requireRegionFits(region, c, kW, kH);
    }
}
