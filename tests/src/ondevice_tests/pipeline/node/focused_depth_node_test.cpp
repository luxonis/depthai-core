#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/node/Depth.hpp"

using namespace dai;

namespace {

// A normalized detection box {xmin, ymin, xmax, ymax}.
struct Box {
    float xmin, ymin, xmax, ymax;
};

std::shared_ptr<Device> requireDefaultDevice(Pipeline& pipeline) {
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        SKIP("Skipping focused-depth test: no device connected.");
    }
    return device;
}

void skipUnlessFocusedDepthSupported(const std::shared_ptr<Device>& device) {
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping focused-depth test: focused depth is RVC4-only.");
    }
    if(device->getStereoPairs().empty()) {
        SKIP("Skipping focused-depth test: device has no stereo pair.");
    }
    if(!device->isNeuralDepthSupported()) {
        SKIP("Skipping focused-depth test: device does not support NeuralDepth.");
    }
}

std::shared_ptr<ImgDetections> makeDetections(const std::vector<Box>& boxes) {
    auto dets = std::make_shared<ImgDetections>();
    std::vector<ImgDetection> list;
    list.reserve(boxes.size());
    for(const auto& b : boxes) {
        ImgDetection det;
        det.label = 0;
        det.confidence = 1.0f;
        det.xmin = b.xmin;
        det.ymin = b.ymin;
        det.xmax = b.xmax;
        det.ymax = b.ymax;
        list.push_back(det);
    }
    dets->detections = std::move(list);
    dets->setTimestamp(std::chrono::steady_clock::now());
    return dets;
}

// Pixel mask (255 inside any detection box, dilated by margin) mirroring the reassembly's
// floor/ceil rounding, on a width x height frame.
cv::Mat boxMask(const std::vector<Box>& boxes, int width, int height, int margin) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
    for(const auto& b : boxes) {
        int x0 = static_cast<int>(std::floor(b.xmin * width)) - margin;
        int y0 = static_cast<int>(std::floor(b.ymin * height)) - margin;
        int x1 = static_cast<int>(std::ceil(b.xmax * width)) + margin;
        int y1 = static_cast<int>(std::ceil(b.ymax * height)) + margin;
        x0 = std::max(0, x0);
        y0 = std::max(0, y0);
        x1 = std::min(width, x1);
        y1 = std::min(height, y1);
        if(x1 > x0 && y1 > y0) {
            mask(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(255);
        }
    }
    return mask;
}

// Runs the focused-depth pipeline, continuously publishing the given detection boxes, and
// returns the received focused depth frame with the most fill (empty Mat if none arrived).
cv::Mat runFocusedDepth(const std::vector<Box>& boxes) {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessFocusedDepthSupported(device);

    auto depth = pipeline.create<node::Depth>();
    depth->build();

    auto detQueue = depth->inputDetections.createInputQueue();
    auto focusedDepthQueue = depth->focusedDepth().createOutputQueue(4, false);
    auto focusedConfQueue = depth->focusedConfidence().createOutputQueue(4, false);

    pipeline.build();
    pipeline.start();

    std::atomic<bool> stop{false};
    std::thread sender([&]() {
        while(!stop.load()) {
            auto dets = makeDetections(boxes);
            dets->setTimestamp(std::chrono::steady_clock::now());
            try {
                detQueue->send(dets);
            } catch(...) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    });

    cv::Mat best;
    int bestFill = -1;
    int framesWithFill = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(40);
    while(std::chrono::steady_clock::now() < deadline && framesWithFill < 5) {
        bool timedOut = false;
        auto fd = focusedDepthQueue->get<ImgFrame>(std::chrono::seconds(2), timedOut);
        if(timedOut || fd == nullptr) {
            continue;
        }
        cv::Mat frame = fd->getFrame();
        if(frame.empty()) {
            continue;
        }
        (void)focusedConfQueue->tryGet<ImgFrame>();
        const int fill = cv::countNonZero(frame.reshape(1) != 0);
        if(fill > 0) {
            ++framesWithFill;
        }
        if(fill > bestFill) {
            bestFill = fill;
            best = frame.clone();
        }
    }

    stop.store(true);
    if(sender.joinable()) {
        sender.join();
    }
    pipeline.stop();

    return best;
}

struct FocusedResult {
    int width = 0;
    int height = 0;
    int totalFill = 0;
    int outsideFill = 0;
    std::vector<int> perBoxFill;
};

FocusedResult analyzeFocused(const cv::Mat& depth, const std::vector<Box>& boxes) {
    REQUIRE_FALSE(depth.empty());
    FocusedResult r;
    r.width = depth.cols;
    r.height = depth.rows;

    cv::Mat nonzero = depth.reshape(1) != 0;  // CV_8UC1, 255 where depth != 0
    r.totalFill = cv::countNonZero(nonzero);

    // Fill must land only inside the detection boxes (plus a 1px rounding margin).
    const cv::Mat mask = boxMask(boxes, r.width, r.height, 1);
    cv::Mat outside;
    cv::bitwise_and(nonzero, ~mask, outside);
    r.outsideFill = cv::countNonZero(outside);

    for(const auto& b : boxes) {
        const cv::Mat single = boxMask({b}, r.width, r.height, 0);
        cv::Mat inside;
        cv::bitwise_and(nonzero, single, inside);
        r.perBoxFill.push_back(cv::countNonZero(inside));
    }
    return r;
}

}  // namespace

TEST_CASE("FocusedDepth: output preserves the left stereo camera instance") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessFocusedDepthSupported(device);
    const auto stereoPair = device->getStereoPairs().front();

    auto depth = pipeline.create<node::Depth>();
    depth->build();
    auto detQueue = depth->inputDetections.createInputQueue();
    auto focusedDepthQueue = depth->focusedDepth().createOutputQueue(4, false);
    auto focusedConfQueue = depth->focusedConfidence().createOutputQueue(4, false);

    pipeline.start();
    std::atomic<bool> stop{false};
    std::thread sender([&]() {
        while(!stop.load()) {
            try {
                detQueue->send(makeDetections({{0.25f, 0.25f, 0.75f, 0.75f}}));
            } catch(...) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    });

    bool depthTimedOut = false;
    auto focusedDepth = focusedDepthQueue->get<ImgFrame>(std::chrono::seconds(20), depthTimedOut);
    bool confidenceTimedOut = false;
    auto focusedConfidence = focusedConfQueue->get<ImgFrame>(std::chrono::seconds(20), confidenceTimedOut);

    stop.store(true);
    sender.join();
    pipeline.stop();

    REQUIRE_FALSE(depthTimedOut);
    REQUIRE(focusedDepth != nullptr);
    REQUIRE(focusedDepth->getInstanceNum() == static_cast<unsigned int>(stereoPair.left));
    REQUIRE_FALSE(confidenceTimedOut);
    REQUIRE(focusedConfidence != nullptr);
    REQUIRE(focusedConfidence->getInstanceNum() == static_cast<unsigned int>(stereoPair.left));
}

TEST_CASE("FocusedDepth: multiple detection regions each get filled, nothing outside them") {
    const std::vector<Box> boxes = {
        {0.10f, 0.15f, 0.30f, 0.45f},
        {0.40f, 0.30f, 0.60f, 0.70f},
        {0.70f, 0.55f, 0.90f, 0.85f},
    };

    const cv::Mat depth = runFocusedDepth(boxes);
    const FocusedResult r = analyzeFocused(depth, boxes);

    REQUIRE(r.totalFill > 0);
    REQUIRE(r.outsideFill == 0);

    // Focused depth for several regions must fill more than one of them (not a single blob).
    int filledBoxes = 0;
    for(int fill : r.perBoxFill) {
        if(fill > 0) {
            ++filledBoxes;
        }
    }
    REQUIRE(filledBoxes >= 2);
}

TEST_CASE("FocusedDepth: fill grows with the number of detection regions") {
    const std::vector<Box> single = {{0.40f, 0.40f, 0.55f, 0.60f}};
    const std::vector<Box> triple = {
        {0.05f, 0.40f, 0.20f, 0.60f},
        {0.40f, 0.40f, 0.55f, 0.60f},
        {0.75f, 0.40f, 0.90f, 0.60f},
    };

    const FocusedResult one = analyzeFocused(runFocusedDepth(single), single);
    REQUIRE(one.totalFill > 0);
    REQUIRE(one.outsideFill == 0);

    const FocusedResult three = analyzeFocused(runFocusedDepth(triple), triple);
    REQUIRE(three.totalFill > 0);
    REQUIRE(three.outsideFill == 0);

    // Three equally sized regions must cover more pixels than one.
    REQUIRE(three.totalFill > one.totalFill);
}

TEST_CASE("FocusedDepth: regions touching the image edges stay clamped inside the frame") {
    const std::vector<Box> boxes = {
        {0.00f, 0.40f, 0.15f, 0.60f},  // left edge
        {0.85f, 0.40f, 1.00f, 0.60f},  // right edge
        {0.40f, 0.00f, 0.60f, 0.15f},  // top edge
        {0.40f, 0.85f, 0.60f, 1.00f},  // bottom edge
        {0.00f, 0.00f, 0.15f, 0.15f},  // top-left corner
    };

    const cv::Mat depth = runFocusedDepth(boxes);
    const FocusedResult r = analyzeFocused(depth, boxes);

    REQUIRE(r.totalFill > 0);
    // Edge/corner regions must not spill outside the frame or beyond their (clamped) boxes.
    REQUIRE(r.outsideFill == 0);
}

TEST_CASE("FocusedDepth: single-model pipeline preserves frame order and largest-region geometry") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessFocusedDepthSupported(device);
    const auto pair = device->getStereoPairs().front();
    auto camera = pipeline.create<node::Camera>()->build(pair.left, std::nullopt, 30.0f);
    auto script = pipeline.create<node::Script>();
    camera->requestOutput({64, 40}, ImgFrame::Type::GRAY8, ImgResizeMode::CROP, 30.0f)->link(script->inputs["frame"]);
    script->setScript(R"(
while True:
    frame = node.io["frame"].get()
    message = ImgDetections()
    phase = frame.getSequenceNum() % 3
    if phase != 0:
        big = ImgDetection()
        big.xmin, big.ymin, big.xmax, big.ymax = (0.1, 0.2, 0.3, 0.7) if phase == 1 else (0.6, 0.2, 0.9, 0.7)
        big.confidence = 1.0
        small = ImgDetection()
        small.xmin, small.ymin, small.xmax, small.ymax = (0.45, 0.2, 0.5, 0.25)
        small.confidence = 1.0
        message.detections = [small, big]
    message.setTimestamp(frame.getTimestamp())
    message.setTimestampDevice(frame.getTimestampDevice())
    message.setSequenceNum(frame.getSequenceNum())
    node.io["detections"].send(message)
)");
    auto depth = pipeline.create<node::Depth>();
    depth->setFocusModels({DeviceModelZoo::NEURAL_DEPTH_192X120});
    depth->setFocusSelectionMode(node::FocusController::SelectionMode::LARGEST);
    depth->build(30.0f);
    script->outputs["detections"].link(depth->inputDetections);
    auto output = depth->focusedDepth().createOutputQueue(4, false);
    auto confidence = depth->focusedConfidence().createOutputQueue(4, false);

    int backends = 0;
    for(const auto& child : depth->getAllNodes()) {
        if(std::dynamic_pointer_cast<node::NeuralDepth>(child)) ++backends;
        if(auto focused = std::dynamic_pointer_cast<node::FocusedDepth>(child)) {
            REQUIRE_THROWS(focused->setFocusModels({DeviceModelZoo::NEURAL_DEPTH_288X180}));
            REQUIRE_THROWS(focused->setFocusSelectionMode(node::FocusController::SelectionMode::ALL));
            REQUIRE_THROWS(focused->setFocusDispatchMode(node::FocusController::DispatchMode::TIME_BUDGET));
        }
    }
    REQUIRE(backends == 1);
    REQUIRE_THROWS(depth->setFocusModels({DeviceModelZoo::NEURAL_DEPTH_288X180}));

    struct StopGuard {
        Pipeline& pipeline;
        ~StopGuard() {
            pipeline.stop();
        }
    } guard{pipeline};
    pipeline.start();
    int phases[3] = {0, 0, 0};
    int filled = 0;
    int64_t previous = -1;
    for(int i = 0; i < 90; ++i) {
        bool timedOut = false;
        auto frame = output->get<ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getSequenceNum() > previous);
        previous = frame->getSequenceNum();
        const int phase = previous % 3;
        ++phases[phase];
        const auto pixels = frame->getFrame();
        if(phase == 0) {
            REQUIRE(cv::countNonZero(pixels) == 0);
        } else {
            const Box box = phase == 1 ? Box{0.1f, 0.2f, 0.3f, 0.7f} : Box{0.6f, 0.2f, 0.9f, 0.7f};
            const auto result = analyzeFocused(pixels, {box});
            REQUIRE(result.outsideFill == 0);
            filled += result.totalFill > 0;
        }
        (void)confidence->tryGetAll<ImgFrame>();
    }
    REQUIRE(phases[0] > 10);
    REQUIRE(phases[1] > 10);
    REQUIRE(phases[2] > 10);
    REQUIRE(filled > 30);
}

TEST_CASE("FocusedDepth modes preserve fresh frames and fuse only the selected region") {
    using Mode = node::FocusController::Mode;
    const auto mode = GENERATE(Mode::ROI, Mode::HOLD, Mode::HYBRID);
    CAPTURE(static_cast<int>(mode));
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessFocusedDepthSupported(device);
    const auto pair = device->getStereoPairs().front();
    auto camera = pipeline.create<node::Camera>()->build(pair.left, std::nullopt, 30.0f);
    auto script = pipeline.create<node::Script>();
    camera->requestOutput({64, 40}, ImgFrame::Type::GRAY8, ImgResizeMode::CROP, 30.0f)->link(script->inputs["frame"]);
    // Synthetic detection gaps are deterministic. The production example has no Script node.
    script->setScript(R"(
while True:
    frame = node.io["frame"].get()
    message = ImgDetections()
    if frame.getSequenceNum() % 8 < 2:
        detection = ImgDetection()
        detection.xmin, detection.ymin, detection.xmax, detection.ymax = 0.35, 0.3, 0.65, 0.7
        detection.confidence = 1.0
        message.detections = [detection]
    message.setTimestamp(frame.getTimestamp())
    message.setTimestampDevice(frame.getTimestampDevice())
    message.setSequenceNum(frame.getSequenceNum())
    node.io["detections"].send(message)
)");
    auto depth = pipeline.create<node::Depth>();
    depth->setFocusMode(mode);
    depth->setFocusHoldFrames(2);
    depth->setFocusModels({DeviceModelZoo::NEURAL_DEPTH_MEDIUM});
    depth->setFocusSelectionMode(node::FocusController::SelectionMode::LARGEST);
    depth->build(30.0f);
    REQUIRE_THROWS(depth->setFocusStereoSize(0, 400));
    REQUIRE_THROWS(depth->setFocusStereoSize(639, 400));
    REQUIRE_THROWS(depth->setFocusStereoSize(480, 300));
    script->outputs["detections"].link(depth->inputDetections);
    auto output = depth->focusedDepth().createOutputQueue(8, false);
    std::shared_ptr<MessageQueue> baseline;
    int stereoCount = 0;
    for(const auto& child : depth->getAllNodes()) {
        if(auto stereo = std::dynamic_pointer_cast<node::StereoDepth>(child)) {
            ++stereoCount;
            baseline = stereo->depth.createOutputQueue(16, false);
        }
    }
    REQUIRE(stereoCount == (mode == Mode::HYBRID ? 1 : 0));
    REQUIRE_THROWS(depth->setFocusMode(Mode::ROI));
    REQUIRE_THROWS(depth->setFocusHoldFrames(3));
    REQUIRE_THROWS(depth->setFocusStereoSize(640, 400));
    struct PipelineStopGuard {
        Pipeline& pipeline;
        ~PipelineStopGuard() {
            pipeline.stop();
        }
    } guard{pipeline};
    pipeline.start();
    std::map<int64_t, std::shared_ptr<ImgFrame>> baseFrames;
    int filled = 0;
    int empty = 0;
    int held = 0;
    int64_t previous = -1;
    auto previousTimestamp = std::chrono::steady_clock::time_point{};
    for(int i = 0; i < 96; ++i) {
        bool timedOut = false;
        auto frame = output->get<ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getSequenceNum() > previous);
        REQUIRE(frame->getTimestamp() > previousTimestamp);
        previous = frame->getSequenceNum();
        previousTimestamp = frame->getTimestamp();
        // Skip initial partial cycle, where no prior detection may have been seen.
        if(previous < 8) continue;
        const int phase = previous % 8;
        const bool enhanced = phase < 2 || (mode == Mode::HOLD && phase < 4);
        const auto pixels = frame->getFrame();
        if(mode == Mode::HYBRID) {
            for(const auto& base : baseline->tryGetAll<ImgFrame>()) baseFrames[base->getSequenceNum()] = base;
            const auto found = baseFrames.find(previous);
            REQUIRE(found != baseFrames.end());
            const auto base = found->second;
            REQUIRE(base->getTimestamp() == frame->getTimestamp());
            cv::Mat expected;
            cv::resize(base->getFrame(), expected, pixels.size(), 0, 0, cv::INTER_NEAREST);
            cv::Mat difference;
            cv::compare(expected, pixels, difference, cv::CMP_NE);
            const auto mask = boxMask({Box{0.35f, 0.3f, 0.65f, 0.7f}}, pixels.cols, pixels.rows, 1);
            cv::Mat outside;
            cv::bitwise_and(difference, ~mask, outside);
            REQUIRE(cv::countNonZero(outside) == 0);
            if(!enhanced) REQUIRE(cv::countNonZero(difference) == 0);
            if(enhanced) filled += cv::countNonZero(difference) > 0;
            REQUIRE(cv::countNonZero(pixels) > 0);
            baseFrames.erase(baseFrames.begin(), baseFrames.upper_bound(previous));
        } else if(enhanced) {
            const auto result = analyzeFocused(pixels, {Box{0.35f, 0.3f, 0.65f, 0.7f}});
            REQUIRE(result.outsideFill == 0);
            filled += result.totalFill > 0;
            if(phase >= 2) held += result.totalFill > 0;
        } else {
            REQUIRE(cv::countNonZero(pixels) == 0);
            ++empty;
        }
    }
    REQUIRE(filled > 10);
    if(mode != Mode::HYBRID) REQUIRE(empty > 10);
    if(mode == Mode::HOLD) REQUIRE(held > 10);
}
