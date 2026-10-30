#pragma once

#include "img_detections_filter_test_harness.hpp"

namespace filtertest {

struct SingleInputCase {
    std::string variant;
    FilterSettings settings;
    std::shared_ptr<dai::ImgDetections> input;
    dai::ImgTransformation outputTransformation = transformation();
    std::vector<ExpectedDetection> expected;
    std::optional<std::string> mask;
    std::size_t maskWidth = 0, maskHeight = 0;
    float tolerance = 1e-4f, angleTolerance = 1e-4f;
    bool unchanged = false;
    std::optional<std::array<float, 4>> outer;
};

inline std::vector<ExpectedDetection> maskDetections() {
    return {{1, 2, 2, 4, .9f, 1}, {4, 2, 2, 4, .25f, 2}, {7, 2, 2, 4, .75f, 1}};
}

inline std::shared_ptr<dai::ImgDetections> maskMessage() {
    auto input = message(transformation(8, 4), maskDetections());
    input->setSegmentationMask(maskBytes("0 0 . 1 1 . 2 2\n0 0 . 1 1 . 2 2\n0 0 . 1 1 . 2 2\n. . . 1 1 . . ."), 8, 4);
    return input;
}

// Shared input AND oracle for V-5; no output is derived from the filter.
inline std::vector<SingleInputCase> sharedCases(const std::string& id) {
    std::vector<SingleInputCase> cases;
    SingleInputCase c;
    c.variant = "default";
    if(id == "B-1") {
        c.expected = {{100, 100, 64, 64, .9f, 1, 0, "one", {{102.4f, 102.4f, .7f}, {128, 128, .6f}}},
                      {300, 100, 100, 40, .3f, 2, 90, "two"},
                      {300, 300, 80, 40, .6f, 3, 20, "three"}};
        c.input = message(transformation(), c.expected);
        c.input->setSegmentationMask(maskBytes("0 1 2 ."), 4, 1);
        c.mask = "0 1 2 .";
        c.maskWidth = 4;
        c.maskHeight = 1;
        c.unchanged = true;
        cases.push_back(c);
    } else if(id == "B-3") {
        std::vector<ExpectedDetection> data;
        for(std::uint32_t label = 1; label <= 5; ++label) data.push_back({100, 100, 64, 64, .9f, label});
        for(int variant = 0; variant < 4; ++variant) {
            c = {};
            c.variant = std::vector<std::string>{"keep and reject", "reject", "keep", "empty keep"}[variant];
            if(variant == 0 || variant == 2) c.settings.keep = std::vector<std::uint32_t>{1, 2, 3};
            if(variant <= 1) c.settings.reject = std::vector<std::uint32_t>{2, 4};
            if(variant == 3) c.settings.keep = std::vector<std::uint32_t>{};
            const std::vector<std::vector<std::size_t>> indices = {{0, 2}, {0, 2, 4}, {0, 1, 2}, {}};
            for(auto i : indices[variant]) c.expected.push_back(data[i]);
            c.input = message(transformation(), data);
            cases.push_back(c);
        }
    } else if(id == "B-4") {
        const std::vector<ExpectedDetection> data = {{64, 64, 64, 64, .5f},
                                                     {192, 64, 128, 64, .75f},
                                                     {320, 64, 64, 64, .4375f},
                                                     {448, 64, 64, 64, .8125f},
                                                     {64, 192, 64, 63, .625f},
                                                     {192, 192, 63, 128, .625f},
                                                     {320, 192, 129, 32, .625f},
                                                     {448, 192, 50, 120, .625f, 0, 90}};
        c.settings.minConfidence = .5f;
        c.settings.maxConfidence = .75f;
        c.settings.minArea = 4096;
        c.settings.maxArea = 8192;
        c.settings.minWidth = 64;
        c.settings.maxWidth = 128;
        c.input = message(transformation(), data);
        c.expected = {data[0], data[1], data[7]};
        cases.push_back(c);
    } else if(id == "B-8") {
        const std::vector<ExpectedDetection> data = {{150, 150, 100, 100},
                                                     {250, 250, 100, 100},
                                                     {260, 200, 100, 100},
                                                     {400, 400, 50, 50},
                                                     {200, 200, 300, 300},
                                                     {200, 200, 260, 20, .9f, 0, 45},
                                                     {200, 200, 260, 20}};
        c.settings.roi = dai::Rect(100, 100, 200, 200, false);
        c.input = message(transformation(), data);
        c.expected = {data[0], data[1], data[5]};
        cases.push_back(c);
    } else if(id == "D-1") {
        const std::vector<ExpectedDetection> data = {{100, 100, 64, 64, .5f}, {300, 100, 64, 64, .75f}, {100, 300, 64, 64, .875f}, {300, 300, 64, 64, .75f}};
        const std::vector<std::vector<std::size_t>> indices = {{1, 2}, {2, 1}, {2, 1, 3}, {2, 1, 3, 0}, {0, 1, 2, 3}};
        for(int variant = 0; variant < 5; ++variant) {
            c = {};
            c.variant = std::vector<std::string>{"K=2", "K=2 sorted", "K=3 sorted", "sorted", "K=10"}[variant];
            if(variant < 3) c.settings.maxDetections = variant == 2 ? 3 : 2;
            if(variant == 4) c.settings.maxDetections = 10;
            if(variant >= 1 && variant <= 3) c.settings.sort = true;
            c.input = message(transformation(), data);
            for(auto i : indices[variant]) c.expected.push_back(data[i]);
            cases.push_back(c);
        }
    } else if(id == "E-1") {
        c.settings.minConfidence = .5f;
        c.input = maskMessage();
        c.outputTransformation = transformation(8, 4);
        const auto data = maskDetections();
        c.expected = {data[0], data[2]};
        c.mask = "0 0 . . . . 1 1\n0 0 . . . . 1 1\n0 0 . . . . 1 1\n. . . . . . . .";
        c.maskWidth = 8;
        c.maskHeight = 4;
        cases.push_back(c);
    } else if(id == "E-2") {
        for(const bool masked : {true, false}) {
            c = {};
            c.variant = masked ? "with mask" : "without mask";
            c.settings.minConfidence = .75f;
            c.outputTransformation = transformation(4, 4);
            c.input = message(c.outputTransformation, {{1, 1, 2, 2, .25f}, {3, 3, 2, 2, .5f}});
            if(masked) {
                c.input->setSegmentationMask(maskBytes("0 0 . .\n0 0 . .\n. . 1 1\n. . 1 1"), 4, 4);
                c.mask = ". . . .\n. . . .\n. . . .\n. . . .";
                c.maskWidth = c.maskHeight = 4;
            }
            cases.push_back(c);
        }
    } else if(id == "R-1") {
        c.settings.reference = transformation();
        c.input = message(scaled(transformation(), .5f), {{64, 64, 32, 32, .9f, 5, 0, "obj"}});
        c.expected = {{128, 128, 64, 64, .9f, 5, 0, "obj"}};
        c.tolerance = c.angleTolerance = .01f;
        c.outer = std::array<float, 4>{.1875f, .1875f, .3125f, .3125f};
        cases.push_back(c);
    } else if(id == "R-3") {
        c.settings.reference = transformation();
        c.input = message(transformation(512, 512, rotation(90)), {{300, 200, 100, 40, .9f, 5, 0, "obj"}});
        c.expected = {{312, 300, 40, 100, .9f, 5, 0, "obj"}};
        c.tolerance = .05f;
        c.angleTolerance = .1f;
        c.outer = std::array<float, 4>{292.f / 512, 250.f / 512, 332.f / 512, 350.f / 512};
        cases.push_back(c);
    } else {
        FAIL("Unknown shared case " << id);
    }
    return cases;
}

inline void runSharedCase(const std::string& id, bool device = false) {
    for(const auto& c : sharedCases(id)) {
        DYNAMIC_SECTION(id << ": " << c.variant) {
            FilterHarness h(c.settings, {"cam"}, device);
            if(device) {
                REQUIRE(h.pipeline.getDefaultDevice() != nullptr);
                if(h.pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("V-5 requires RVC4");
                h.setRunOnHost(false);
            }
            const MessageSnapshot before(*c.input);
            h.pipeline.start();
            if(device) REQUIRE_FALSE(h.runOnHost());
            h.send(c.input);
            const auto output = h.receive();
            requireOutput(*output, c.outputTransformation, c.expected, c.tolerance, c.angleTolerance);
            requireMetadata(*output, *c.input);
            if(c.mask)
                requireMask(*output, c.maskWidth, c.maskHeight, *c.mask);
            else
                REQUIRE_FALSE(output->getMaskData().has_value());
            before.requireEqual(*c.input);
            if(c.unchanged) before.requireEqual(*output);
            if(!c.settings.reference) {
                for(std::size_t i = 0; i < c.expected.size(); ++i) {
                    REQUIRE(dai::utility::serialize(output->detections[i]) == dai::utility::serialize(detection(c.outputTransformation, c.expected[i])));
                }
            }
            if(c.outer) {
                REQUIRE(output->detections.size() == 1);
                const auto& d = output->detections[0];
                REQUIRE(d.getBoundingBox().isNormalized());
                const std::array<float, 4> fields = {d.xmin, d.ymin, d.xmax, d.ymax};
                for(std::size_t i = 0; i < fields.size(); ++i) REQUIRE_THAT(fields[i], Catch::Matchers::WithinAbs((*c.outer)[i], c.tolerance / 512));
            }
        }
    }
}

}  // namespace filtertest
