/**
 * @file Depth.cpp
 * @brief Implementation of dai::node::Depth: stereo camera resolution, backend creation, and output aliasing.
 */
#include "depthai/pipeline/node/Depth.hpp"

#include <cstring>

#include "depthai/common/StereoPair.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {
namespace {

/// Default ISP stream rate when requesting Camera outputs for stereo feeding Depth.
constexpr float kDefaultIspFps = 30.f;

/**
 * @brief Detaches a node from the pipeline root and attaches it under a DeviceNodeGroup parent.
 */
void adoptFromPipeline(Pipeline& pipeline, Depth& parent, const std::shared_ptr<Node>& child) {
    pipeline.remove(child);
    parent.add(child);
}

/**
 * @brief Creates a device node, removes it from the pipeline map, and adopts it under Depth.
 * @tparam T Node type (e.g. StereoDepth, NeuralDepth).
 */
template <typename T>
std::shared_ptr<T> createAdopted(Pipeline& pipeline, Depth& parent) {
    auto child = pipeline.create<T>();
    adoptFromPipeline(pipeline, parent, child);
    return child;
}

/**
 * @brief Finds existing Camera nodes matching the left/right sockets of a stereo pair.
 * @return Pair of (left, right) cameras; either side may be null if not present in the pipeline.
 */
std::pair<std::shared_ptr<Camera>, std::shared_ptr<Camera>> findCamerasForPair(const Pipeline& pipeline, const StereoPair& pair) {
    std::shared_ptr<Camera> left;
    std::shared_ptr<Camera> right;
    for(const auto& node : pipeline.getAllNodes()) {
        if(std::strcmp(node->getName(), Camera::NAME) != 0) {
            continue;
        }
        auto cam = std::static_pointer_cast<Camera>(node);
        const auto socket = cam->getBoardSocket();
        if(socket == pair.left) {
            left = std::move(cam);
        } else if(socket == pair.right) {
            right = std::move(cam);
        }
    }
    return {left, right};
}

/**
 * @brief Returns the first stereo pair advertised by the device.
 * @throws std::runtime_error If the device exposes no stereo pairs (via DAI_CHECK_V).
 */
StereoPair requireFirstStereoPair(const std::shared_ptr<Device>& device) {
    const auto pairs = device->getStereoPairs();
    DAI_CHECK_V(!pairs.empty(), "Device has no stereo camera pair for Depth node.");
    return pairs[0];
}

/**
 * @brief Ensures left/right cameras exist for the pair and returns their ISP outputs at kDefaultIspFps.
 *
 * Reuses existing Camera nodes on the pair sockets when found; otherwise creates them and adopts under parent.
 * @return Pointers to left and right Camera ISP outputs suitable for StereoDepth::build / NeuralDepth::build.
 */
std::pair<Node::Output*, Node::Output*> ensureStereoIspOutputs(Pipeline& pipeline, Depth& parent, const StereoPair& pair) {
    auto [leftCam, rightCam] = findCamerasForPair(pipeline, pair);
    if(!leftCam) {
        leftCam = pipeline.create<Camera>()->build(pair.left);
        adoptFromPipeline(pipeline, parent, leftCam);
    }
    if(!rightCam) {
        rightCam = pipeline.create<Camera>()->build(pair.right);
        adoptFromPipeline(pipeline, parent, rightCam);
    }
    auto* lo = leftCam->requestIspOutput(kDefaultIspFps);
    auto* ro = rightCam->requestIspOutput(kDefaultIspFps);
    DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera ISP output request failed.");
    return {lo, ro};
}

}  // namespace

Depth::Depth(const std::shared_ptr<Device>& device) : DeviceNodeGroup(device) {}

void Depth::buildInternal() {
    // Depth defers wiring to build(); nothing to do at construction time.
}

std::shared_ptr<Depth> Depth::build(DeviceModelZoo neuralModel) {
    DAI_CHECK_V(!built_, "Depth::build() was already called.");
    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device.");

    auto pipeline = getParentPipeline();
    DAI_CHECK_V(pipeline.impl() != nullptr, "Depth node must be added to a pipeline before build().");

    const Platform platform = device->getPlatform();
    const auto pair = requireFirstStereoPair(device);
    auto [ispLeft, ispRight] = ensureStereoIspOutputs(pipeline, *this, pair);

    if(platform == Platform::RVC4) {
        neural_ = createAdopted<NeuralDepth>(pipeline, *this);
        neural_->build(*ispLeft, *ispRight, neuralModel);
        depthOut_ = &neural_->depth;
        confidenceOut_ = &neural_->confidence;
    } else {
        stereo_ = createAdopted<StereoDepth>(pipeline, *this);
        stereo_->build(*ispLeft, *ispRight, StereoDepth::PresetMode::DEFAULT);
        depthOut_ = &stereo_->depth;
        confidenceOut_ = &stereo_->confidenceMap;
    }

    built_ = true;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

Node::Output& Depth::depth() {
    DAI_CHECK_V(built_ && depthOut_ != nullptr, "Depth::build() must be called before accessing outputs.");
    return *depthOut_;
}

Node::Output& Depth::confidence() {
    DAI_CHECK_V(built_ && confidenceOut_ != nullptr, "Depth::build() must be called before accessing outputs.");
    return *confidenceOut_;
}

}  // namespace node
}  // namespace dai
