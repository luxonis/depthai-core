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

namespace {

FocusController::Crop cropRect(int x, int y, int w, int h) {
    // Detection box == crop box for merge tests (padding is irrelevant to the union logic).
    return FocusController::Crop{x, y, w, h, static_cast<float>(x), static_cast<float>(y), static_cast<float>(w), static_cast<float>(h)};
}

int totalDets(const std::vector<FocusController::MergedCrop>& merged) {
    int n = 0;
    for(const auto& m : merged) {
        n += static_cast<int>(m.dets.size());
    }
    return n;
}

// No two merged crops may overlap after merging.
void requireNoOverlap(const std::vector<FocusController::MergedCrop>& merged) {
    for(std::size_t i = 0; i < merged.size(); ++i) {
        for(std::size_t j = i + 1; j < merged.size(); ++j) {
            const auto& a = merged[i];
            const auto& b = merged[j];
            const bool overlap = a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
            REQUIRE_FALSE(overlap);
        }
    }
}

}  // namespace

TEST_CASE("FocusController::mergeCrops: overlapping crops become one inference, disjoint stay separate", "[FocusController]") {
    SECTION("two overlapping crops merge into their union and keep both detections") {
        const std::vector<FocusController::Crop> crops = {cropRect(100, 100, 200, 200), cropRect(250, 150, 200, 200)};
        const auto merged = FocusController::mergeCrops(crops);
        REQUIRE(merged.size() == 1);
        REQUIRE(merged[0].dets.size() == 2);
        // Union rectangle covers both inputs.
        REQUIRE(merged[0].x == 100);
        REQUIRE(merged[0].y == 100);
        REQUIRE(merged[0].x + merged[0].w == 450);
        REQUIRE(merged[0].y + merged[0].h == 350);
        requireNoOverlap(merged);
    }
    SECTION("disjoint crops are not merged") {
        const std::vector<FocusController::Crop> crops = {cropRect(0, 0, 100, 100), cropRect(500, 500, 100, 100)};
        const auto merged = FocusController::mergeCrops(crops);
        REQUIRE(merged.size() == 2);
        REQUIRE(merged[0].dets.size() == 1);
        REQUIRE(merged[1].dets.size() == 1);
    }
    SECTION("merely touching (shared edge, no overlap) crops are not merged") {
        const std::vector<FocusController::Crop> crops = {cropRect(0, 0, 100, 100), cropRect(100, 0, 100, 100)};
        const auto merged = FocusController::mergeCrops(crops);
        REQUIRE(merged.size() == 2);
    }
    SECTION("transitive overlap: A-B and B-C overlap but A-C do not; all fold into one") {
        const std::vector<FocusController::Crop> crops = {cropRect(0, 0, 120, 100), cropRect(100, 0, 120, 100), cropRect(200, 0, 120, 100)};
        const auto merged = FocusController::mergeCrops(crops);
        REQUIRE(merged.size() == 1);
        REQUIRE(merged[0].dets.size() == 3);
        REQUIRE(merged[0].x == 0);
        REQUIRE(merged[0].x + merged[0].w == 320);
        requireNoOverlap(merged);
    }
    SECTION("no detection is ever dropped and the result has no overlaps") {
        const std::vector<FocusController::Crop> crops = {
            cropRect(10, 10, 100, 100),
            cropRect(80, 40, 100, 100),   // overlaps the first
            cropRect(400, 300, 50, 50),   // isolated
            cropRect(420, 320, 60, 60),   // overlaps the previous
            cropRect(900, 700, 40, 40),   // isolated
        };
        const auto merged = FocusController::mergeCrops(crops);
        REQUIRE(totalDets(merged) == static_cast<int>(crops.size()));
        REQUIRE(merged.size() == 3);
        requireNoOverlap(merged);
    }
    SECTION("empty input yields no crops") {
        REQUIRE(FocusController::mergeCrops({}).empty());
    }
}

TEST_CASE("FocusController::mergeCrops: overlapping detections from computeCrops fold into fewer inferences", "[FocusController]") {
    // Two detections close enough that their disparity-padded crops overlap must merge, while every
    // detection box is still individually covered by the merged crop it belongs to.
    const std::vector<std::array<float, 4>> boxes = {
        box(0.40f, 0.40f, 0.50f, 0.60f),
        box(0.52f, 0.40f, 0.62f, 0.60f),
    };
    const auto crops = FocusController::computeCrops(kW, kH, boxes);
    REQUIRE(crops.size() == 2);
    const auto merged = FocusController::mergeCrops(crops);
    REQUIRE(merged.size() < crops.size());
    REQUIRE(totalDets(merged) == static_cast<int>(crops.size()));
    for(const auto& m : merged) {
        for(const auto& det : m.dets) {
            // Each detection box lies within the merged crop it was folded into.
            REQUIRE(det[0] >= static_cast<float>(m.x) - 1.0f);
            REQUIRE(det[1] >= static_cast<float>(m.y) - 1.0f);
            REQUIRE(det[0] + det[2] <= static_cast<float>(m.x + m.w) + 1.0f);
            REQUIRE(det[1] + det[3] <= static_cast<float>(m.y + m.h) + 1.0f);
        }
    }
}

TEST_CASE("FocusController::selectTier: routes a crop to the smallest tier that fits it", "[FocusController]") {
    // Tiers ascend in size; a crop goes to the first tier whose model dimensions cover it.
    const auto& tiers = FocusController::kTiers;
    REQUIRE(FocusController::kNumTiers == 3);

    SECTION("a crop that fits the smallest tier uses tier 0") {
        REQUIRE(FocusController::selectTier(tiers[0].w, tiers[0].h) == 0);
        REQUIRE(FocusController::selectTier(64, 48) == 0);
    }
    SECTION("a crop larger than tier 0 but within tier 1 uses tier 1") {
        REQUIRE(FocusController::selectTier(tiers[0].w + 1, tiers[0].h + 1) == 1);
        REQUIRE(FocusController::selectTier(tiers[1].w, tiers[1].h) == 1);
    }
    SECTION("a crop larger than tier 1 uses the largest tier") {
        REQUIRE(FocusController::selectTier(tiers[1].w + 1, tiers[1].h + 1) == 2);
        REQUIRE(FocusController::selectTier(tiers[2].w, tiers[2].h) == 2);
    }
    SECTION("a crop larger than every tier still clamps to the largest tier") {
        REQUIRE(FocusController::selectTier(tiers[2].w + 500, tiers[2].h + 500) == FocusController::kNumTiers - 1);
    }
    SECTION("tiers are strictly ascending in both dimensions") {
        for(int t = 1; t < FocusController::kNumTiers; ++t) {
            REQUIRE(tiers[t].w > tiers[t - 1].w);
            REQUIRE(tiers[t].h > tiers[t - 1].h);
        }
    }
}

TEST_CASE("FocusController::selectLargest: keeps only the largest detection", "[FocusController]") {
    const std::vector<std::array<float, 4>> boxes = {
        box(0.05f, 0.05f, 0.20f, 0.20f),
        box(0.20f, 0.10f, 0.80f, 0.70f),
        box(0.10f, 0.10f, 0.40f, 0.90f),
    };
    const auto largest = FocusController::selectLargest(boxes);
    REQUIRE(largest.size() == 1);
    REQUIRE(largest[0] == boxes[1]);
    REQUIRE(FocusController::selectLargest({}).empty());
}

TEST_CASE("FocusController::selectTier: uses configurable tier dimensions", "[FocusController]") {
    const std::array<FocusController::Tier, FocusController::kNumTiers> tiers{{
        {dai::DeviceModelZoo::NEURAL_DEPTH_NANO, 384, 240},
        {dai::DeviceModelZoo::NEURAL_DEPTH_SMALL, 480, 300},
        {dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM, 576, 360},
    }};
    REQUIRE(FocusController::selectTier(tiers, 3, 385, 241) == 1);
    REQUIRE(FocusController::selectTier(tiers, 3, 481, 301) == 2);
    REQUIRE(FocusController::selectTier(tiers, 3, 900, 500) == 2);
    REQUIRE(FocusController::selectTier(tiers, 1, 500, 500) == 0);
}

TEST_CASE("FocusController::orderCropsByArea: largest crops are dispatched first", "[FocusController]") {
    const std::vector<FocusController::MergedCrop> crops = {
        {0, 0, 20, 20, {}},
        {0, 0, 80, 10, {}},
        {0, 0, 30, 30, {}},
    };
    const auto ordered = FocusController::orderCropsByArea(crops);
    REQUIRE(ordered.size() == crops.size());
    REQUIRE(ordered[0].w * ordered[0].h == 900);
    REQUIRE(ordered[1].w * ordered[1].h == 800);
    REQUIRE(ordered[2].w * ordered[2].h == 400);
}

TEST_CASE("FocusController::depthFocalScale: makes depth crop-size invariant", "[FocusController]") {
    // Full rectified frame focal (1280-wide) and neural backend output width.
    constexpr float fxFull = 570.42f;
    constexpr int outW = 480;

    SECTION("no-op when the frame intrinsic already matches the crop's true focal") {
        // When the crop frame carries the geometrically correct focal, no correction is needed.
        for(const int cropW : {480, 576, 768, 1024, 1280}) {
            const float fxCorrect = fxFull * static_cast<float>(outW) / static_cast<float>(cropW);
            REQUIRE(FocusController::depthFocalScale(fxFull, fxCorrect, outW, cropW) == Catch::Approx(1.0f));
        }
    }

    SECTION("corrects a frozen/stale focal so corrected depth is constant across crop widths") {
        // Firmware used a fixed focal (stuck at the first crop's value, cropW=576 here). The
        // backend depth then scales like cropW, but applying depthFocalScale must cancel that so
        // the corrected depth is the same for every crop width viewing the same surface.
        const float fxUsed = fxFull * static_cast<float>(outW) / 576.0f;  // 475.35
        const float trueDepthMm = 3300.0f;
        for(const int cropW : {576, 768, 1024, 1280}) {
            // Firmware depth for this crop: true * (fxUsed / fxCorrect) = true * cropW / 576.
            const float fxCorrect = fxFull * static_cast<float>(outW) / static_cast<float>(cropW);
            const float firmwareDepth = trueDepthMm * fxUsed / fxCorrect;
            const float corrected = firmwareDepth * FocusController::depthFocalScale(fxUsed == 0 ? 1.0f : fxFull, fxUsed, outW, cropW);
            REQUIRE(corrected == Catch::Approx(trueDepthMm).epsilon(0.001));
        }
    }

    SECTION("returns 1.0 for degenerate inputs") {
        REQUIRE(FocusController::depthFocalScale(0.0f, 475.0f, outW, 576) == Catch::Approx(1.0f));
        REQUIRE(FocusController::depthFocalScale(fxFull, 0.0f, outW, 576) == Catch::Approx(1.0f));
        REQUIRE(FocusController::depthFocalScale(fxFull, 475.0f, 0, 576) == Catch::Approx(1.0f));
        REQUIRE(FocusController::depthFocalScale(fxFull, 475.0f, outW, 0) == Catch::Approx(1.0f));
        REQUIRE(FocusController::depthFocalScale(-1.0f, 475.0f, outW, 576) == Catch::Approx(1.0f));
    }
}
