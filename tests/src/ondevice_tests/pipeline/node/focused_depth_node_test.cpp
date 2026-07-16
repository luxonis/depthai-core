#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
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
