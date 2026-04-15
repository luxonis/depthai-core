#include "depthai/pipeline/node/Depth.hpp"

#include <cstring>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {
namespace {

constexpr float kStereoMonoFps = 30.f;
constexpr std::pair<uint32_t, uint32_t> kStereoDepthMonoSize{640, 400};
/// Fixed NeuralDepth zoo model for RVC4 (``Depth`` does not take a per-pipeline model yet).
constexpr DeviceModelZoo kRvc4NeuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;

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

StereoPair requireFirstStereoPair(const std::shared_ptr<Device>& device) {
    const auto pairs = device->getStereoPairs();
    DAI_CHECK_V(!pairs.empty(), "Device has no stereo camera pair for Depth node.");
    return pairs[0];
}

}  // namespace

Depth::Depth(const std::shared_ptr<Device>& device) : DeviceNodeGroup(device) {
    DAI_CHECK_V(device != nullptr, "Depth node requires a device.");
    if(device->getPlatform() == Platform::RVC4) {
        neuralBackend_ = std::make_unique<::dai::Subnode<NeuralDepth>>(*this, "neuralDepth");
        depthOut_ = &(*neuralBackend_)->depth;
        confidenceOut_ = &(*neuralBackend_)->confidence;
    } else {
        stereoBackend_ = std::make_unique<::dai::Subnode<StereoDepth>>(*this, "stereoDepth");
        depthOut_ = &(*stereoBackend_)->depth;
        confidenceOut_ = &(*stereoBackend_)->confidenceMap;
    }
}

void Depth::buildInternal() {
    if(graphBuilt_) {
        return;
    }
    if(parent.lock() == nullptr) {
        return;
    }

    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device.");

    Pipeline pipeline = getParentPipeline();
    DAI_CHECK_V(pipeline.impl() != nullptr, "Depth node must be part of a pipeline.");

    const auto platform = device->getPlatform();
    const auto pair = requireFirstStereoPair(device);

    std::pair<uint32_t, uint32_t> monoSize;
    if(platform == Platform::RVC4) {
        const auto is = NeuralDepth::getInputSize(kRvc4NeuralDepthModel);
        monoSize = {static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
    } else {
        monoSize = kStereoDepthMonoSize;
    }

    auto [leftOut, rightOut] = ensureStereoCameraOutputs(pipeline, pair, monoSize, kStereoMonoFps);

    if(platform == Platform::RVC4) {
        DAI_CHECK_V(neuralBackend_ != nullptr, "NeuralDepth subnode missing on RVC4.");
        (*neuralBackend_)->build(*leftOut, *rightOut, kRvc4NeuralDepthModel);
    } else {
        DAI_CHECK_V(stereoBackend_ != nullptr, "StereoDepth subnode missing.");
        (*stereoBackend_)->build(*leftOut, *rightOut, StereoDepth::PresetMode::DEFAULT);
    }

    graphBuilt_ = true;
}

std::pair<Node::Output*, Node::Output*> Depth::ensureStereoCameraOutputs(Pipeline& pipeline,
                                                                        const StereoPair& pair,
                                                                        std::pair<uint32_t, uint32_t> frameSize,
                                                                        float monoFps) {
    auto [leftFound, rightFound] = findCamerasForPair(pipeline, pair);

    std::shared_ptr<Camera> left = leftFound;
    std::shared_ptr<Camera> right = rightFound;

    if(!left) {
        left = pipeline.create<Camera>()->build(pair.left);
    }
    if(!right) {
        right = pipeline.create<Camera>()->build(pair.right);
    }

    auto* lo = left->requestOutput(frameSize, std::nullopt, ImgResizeMode::CROP, monoFps);
    auto* ro = right->requestOutput(frameSize, std::nullopt, ImgResizeMode::CROP, monoFps);
    DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera stereo output request failed.");
    return {lo, ro};
}

Node::Output& Depth::depth() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(depthOut_ != nullptr, "Depth backend output missing.");
    return *depthOut_;
}

Node::Output& Depth::confidence() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(confidenceOut_ != nullptr, "Depth backend confidence output missing.");
    return *confidenceOut_;
}

}  // namespace node
}  // namespace dai
