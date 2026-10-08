#pragma once

#include <map>
#include <thread>

#include "depthai/depthai.hpp"
#include "img_detections_filter_test_fixtures.hpp"

namespace filtertest {

using Mode = dai::ImgDetectionsFilterConfig::OverlapMode;

struct FilterSettings {
    std::optional<std::vector<std::uint32_t>> keep, reject;
    std::optional<float> minConfidence, maxConfidence, minArea, maxArea, minWidth, maxWidth, minHeight, maxHeight;
    std::optional<dai::Rect> roi;
    std::optional<std::uint32_t> maxDetections;
    std::optional<bool> sort;
    std::optional<Mode> mode;
    std::optional<float> iou;
    std::optional<dai::ImgTransformation> reference;
};

inline std::shared_ptr<dai::ImgDetectionsFilterConfig> makeConfig(const FilterSettings& s) {
    auto c = std::make_shared<dai::ImgDetectionsFilterConfig>();
    // This public API exposes fields for the options without setters/getters.
    c->labelsToKeep = s.keep;
    c->labelsToReject = s.reject;
    if(s.minConfidence || s.maxConfidence) c->setConfidenceRange(s.minConfidence.value_or(0), s.maxConfidence.value_or(c->maxConfidence));
    if(s.minArea || s.maxArea) c->setSizeRange(s.minArea.value_or(0), s.maxArea.value_or(c->maxArea));
    if(s.minWidth || s.maxWidth) c->setWidthRange(s.minWidth.value_or(0), s.maxWidth.value_or(c->maxWidth));
    if(s.minHeight || s.maxHeight) c->setHeightRange(s.minHeight.value_or(0), s.maxHeight.value_or(c->maxHeight));
    c->regionOfInterest = s.roi;
    c->maxDetections = s.maxDetections;
    if(s.sort) c->sortByConfidence = *s.sort;
    if(s.mode) c->overlapMode = *s.mode;
    if(s.iou) c->overlapIouThreshold = *s.iou;
    c->reference = s.reference;
    return c;
}

inline FilterSettings nondefaultSettings() {
    FilterSettings s;
    s.keep = std::vector<std::uint32_t>{1, 2, 3};
    s.reject = std::vector<std::uint32_t>{2, 4};
    s.minConfidence = .2f;
    s.maxConfidence = .8f;
    s.minArea = 10;
    s.maxArea = 500;
    s.minWidth = 2;
    s.maxWidth = 40;
    s.minHeight = 3;
    s.maxHeight = 50;
    s.roi = dai::Rect(1, 2, 100, 120, false);
    s.maxDetections = 7;
    s.sort = true;
    s.mode = Mode::AVERAGE;
    s.iou = .6f;
    s.reference = cropped(transformation(), 4, 8, 256, 256);
    return s;
}

class FilterHarness {
   public:
    dai::Pipeline pipeline;

    explicit FilterHarness(const FilterSettings& settings = {},
                           const std::vector<std::string>& keys = {"cam"},
                           bool device = false,
                           bool referenceLinked = false,
                           bool synchronized = false)
        : pipeline(device), node(pipeline.create<dai::node::ImgDetectionsFilter>()) {
        *node->initialConfig = *makeConfig(settings);
        if(synchronized) {
            auto sync = pipeline.create<dai::node::Sync>();
            auto demux = pipeline.create<dai::node::MessageDemux>();
            sync->setRunOnHost(true);
            demux->setRunOnHost(true);
            sync->out.link(demux->input);
            for(const auto& key : keys) {
                demux->outputs[key].setPossibleDatatypes({{dai::DatatypeEnum::ImgDetections, false}});
                demux->outputs[key].link(node->inputs[key]);
                inputs[key] = sync->inputs[key].createInputQueue();
            }
        } else {
            for(const auto& key : keys) inputs[key] = node->inputs[key].createInputQueue();
        }
        if(referenceLinked) referenceQueue = node->inputReference.createInputQueue();
        output = node->out.createOutputQueue();
    }

    void send(const std::shared_ptr<dai::ImgDetections>& msg, const std::string& key = "cam") {
        inputs.at(key)->send(msg);
    }

    void configure(const FilterSettings& settings) {
        node->inputConfig.send(makeConfig(settings));
    }

    void reference(const dai::ImgTransformation& t) {
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->transformation = t;
        node->inputReference.send(frame);
    }

    void linkReference(dai::Node::Output& source) {
        source.link(node->inputReference);
    }

    void accessUnlinked(const std::string& key) {
        unlinked = &node->inputs[key];
    }

    void linkAccessedInput() {
        REQUIRE(unlinked != nullptr);
        unlinked->createInputQueue();
    }

    void setRunOnHost(bool host) {
        node->setRunOnHost(host);
    }

    bool runOnHost() const {
        return node->runOnHost();
    }

    std::shared_ptr<dai::ImgDetections> receive() {
        bool timedOut = false;
        auto result = output->get<dai::ImgDetections>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(result != nullptr);
        return result;
    }

    void requireNoOutput() {
        bool timedOut = false;
        auto result = output->get<dai::ImgDetections>(std::chrono::milliseconds(200), timedOut);
        REQUIRE(timedOut);
        REQUIRE(result == nullptr);
    }

    void requireStopped() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while(pipeline.isRunning() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        REQUIRE_FALSE(pipeline.isRunning());
    }

   private:
    std::shared_ptr<dai::node::ImgDetectionsFilter> node;
    std::map<std::string, std::shared_ptr<dai::InputQueue>> inputs;
    std::shared_ptr<dai::InputQueue> referenceQueue;
    std::shared_ptr<dai::MessageQueue> output;
    dai::Node::Input* unlinked = nullptr;  // Owned by node; captured before build.
};

}  // namespace filtertest
